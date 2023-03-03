package frc.robot.commands.balancing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class DefaultBalancerCommand extends BaseBalancerCommand {
    private final PIDController drivePID;
    private final PIDController turnPID;
    private final Timer timer;

    private double returnDrivePower; // drive power to be returned to DT
    private double returnAngularPower; // angular power to return to DT (for heading correction)

    private final boolean reversed;

    private boolean reachedStation;
    private boolean passedCenter;
    private boolean balanced;

    private double initialHeading;
    private double oldPitch;
    private double currentPitch;
    private double deltaAngle;

    public DefaultBalancerCommand(BaseDrivetrain driveSubsystem) {
        this(driveSubsystem, false);
    }

    /**
     * Constructs a default balancer command from a drive subsystem and a boolean indicating
     * whether to balance from the front or back.
     * 
     * @param driveSubsystem The drive subsystem.
     * @param reversed Whether to balance driving forwards or backwards. Defaults to backwards; pass `true` to balance forwards.
     */
    public DefaultBalancerCommand(BaseDrivetrain driveSubsystem, boolean reversed) {
        super(driveSubsystem);

        drivePID = new PIDController(0.0072, 0.0, 0.0); // no deriv successful
        turnPID = new PIDController(0.1 / 5,0.0, 0.0); // kP = max pwr / max err
        timer = new Timer();

        this.reversed = reversed;
    }

    @Override
    public void initialize() {
        System.out.println("------------------- Balancer initialized -------------------");
        initialHeading = ahrs.getCompassHeading();
        reachedStation = false;
        passedCenter = false;
        balanced = false;
    }

    @Override
    public void execute() {
        returnAngularPower = turnPID.calculate(ahrs.getCompassHeading(), initialHeading); // correct angle of approach

        if (!reachedStation) {
            returnDrivePower = -0.80;
            if (ahrs.getPitch() <= -15.0) {
                reachedStation = true;
                returnDrivePower = -0.17;
            } //.15 successful
        } else {
            currentPitch = ahrs.getPitch();
            deltaAngle = Math.abs(currentPitch - oldPitch); // calc magnitude of angular acceleration based on delta angle over time

            if (!passedCenter) {
                passedCenter = ahrs.getPitch() >= -5.0; // <= 1.0 worked 
            } else {
                returnDrivePower = -1 * drivePID.calculate(ahrs.getPitch(), 0);
                System.out.println(returnDrivePower);
                if (Math.abs(ahrs.getPitch()) <= 1.0 && deltaAngle <= 0.1) balanced = true;
            }
        }

        // TODO: better name
        double actualDrivePower = reversed ? -returnDrivePower : returnDrivePower;
        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            ((BaseSwerveSubsystem) driveSubsystem).setDrivePowers(actualDrivePower, 0.0, 0.0, true);
        } else {
            driveSubsystem.setDrivePowers(actualDrivePower);
        }

        oldPitch = currentPitch; // set the current angle to old angle so it is accessible for next cycle     
    }

    @Override
    public boolean isFinished() {
        return reachedStation && balanced;
        // return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(0.20);
    }
}
