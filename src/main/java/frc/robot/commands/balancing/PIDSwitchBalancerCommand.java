package frc.robot.commands.balancing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class PIDSwitchBalancerCommand extends BaseBalancerCommand {
    private final PIDController roughPID;
    private final PIDController finePID;
    private final PIDController turnPID;
    private final Timer timer;

    private double returnDrivePower; // drive power to be returned to DT
    private double returnAngularPower; // angular power to return to DT (for heading correction)

    private double initialHeading;
    private double oldPitch;
    private double currentPitch;
    private double deltaAngle;

    private boolean reachedStation;
    private boolean passedCenter;
    private boolean waiting;

    private boolean fine; 
    private boolean balanced;

    public PIDSwitchBalancerCommand(BaseDrivetrain driveSubsystem) {
        super(driveSubsystem);

        roughPID = new PIDController(0.5 / 35, 0.0, 0.0); 
        finePID = new PIDController(0.25 / 22, 0.0, 0.0); 
        turnPID = new PIDController(0.1 / 5,0.0, 0.0); // kP = max pwr / max err

        timer = new Timer();
    }

    @Override
    public void initialize() {
        System.out.println("------------------- Balancer initialized -------------------");
        initialHeading = ahrs.getCompassHeading();
        reachedStation = false;
        passedCenter = false;
        balanced = false;
        waiting = false;
    }

    @Override
    public void execute() {
        returnAngularPower = turnPID.calculate(ahrs.getCompassHeading(), initialHeading); // correct angle of approach

        if (!reachedStation) {
            returnDrivePower = -0.80;
            reachedStation = ahrs.getPitch() <= -15.0;
        } else {
            currentPitch = ahrs.getPitch();
            deltaAngle = Math.abs(currentPitch - oldPitch);
            fine = ahrs.getPitch() >= -11.0 && deltaAngle <= 0.25;

            if (fine) returnDrivePower = -1 * finePID.calculate(ahrs.getPitch(), 0);
            else returnDrivePower = -1 * roughPID.calculate(ahrs.getPitch(), 0);

            System.out.println("DeltaAngle" + deltaAngle);

            if (Math.abs(ahrs.getPitch()) <= 2.0 && deltaAngle <= 0.05) {
                balanced = true;
            }
        }

        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            ((BaseSwerveSubsystem) driveSubsystem).setDrivePowers(returnDrivePower, 0.0, 0.0, true);
        } else driveSubsystem.setDrivePowers(returnDrivePower);

        oldPitch = currentPitch; // set the current angle to old angle so it is accessible for next cycle     
    }

    @Override
    public boolean isFinished() {
        return reachedStation && balanced;
        // return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(0.20);
    }
}
