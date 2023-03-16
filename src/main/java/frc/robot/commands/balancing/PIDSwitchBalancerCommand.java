package frc.robot.commands.balancing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class PIDSwitchBalancerCommand extends BaseBalancerCommand {
    private final PIDController roughPID;
    private final PIDController finePID;
    private final Timer timer;

    private double returnDrivePower; // drive power to be returned to DT
    private double prevPitchDegs;

    private boolean reachedStation;
    private boolean balanced;

    public PIDSwitchBalancerCommand(BaseDrivetrain driveSubsystem) {
        super(driveSubsystem);

        roughPID = new PIDController(0.5 / 35, 0.0, 0.0); 
        finePID = new PIDController(0.25 / 22, 0.0, 0.0); 

        timer = new Timer();
    }

    @Override
    public void initialize() {
        System.out.println("------------------- Balancer initialized -------------------");
        reachedStation = false;
        balanced = false;
    }

    @Override
    public void execute() {
        double currentPitchDegs = ahrs.getPitch();

        if (!reachedStation) {
            returnDrivePower = -0.80;
            reachedStation = currentPitchDegs <= - 7.0;
        } else {
            double deltaPitchDegs = Math.abs(currentPitchDegs - prevPitchDegs);
            boolean fine = currentPitchDegs >= -11.0 && deltaPitchDegs <= 0.25;

            returnDrivePower = fine 
                ? -1 * finePID.calculate(currentPitchDegs, 0)
                : -1 * roughPID.calculate(currentPitchDegs, 0);

            // System.out.println("DeltaAngle" + deltaPitchDegs);

            if (Math.abs(currentPitchDegs) <= 2.0 && deltaPitchDegs <= 0.05) {
                balanced = true;
            }
        }

        // if (driveSubsystem instanceof BaseSwerveSubsystem) {
        //     ((BaseSwerveSubsystem) driveSubsystem).setDrivePowers(returnDrivePower, 0.0, 0.0, true);
        // } else 
        driveSubsystem.setDrivePowers(returnDrivePower);

        prevPitchDegs = currentPitchDegs;
    }

    @Override
    public boolean isFinished() {
        return reachedStation && balanced;
        // return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(0.20);
    }
}
