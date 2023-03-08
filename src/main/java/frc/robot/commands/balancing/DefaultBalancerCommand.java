package frc.robot.commands.balancing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class DefaultBalancerCommand extends BaseBalancerCommand {
    private final PIDController drivePID;
    private final Timer timer;

    private double returnDrivePower; // drive power to be returned to DT
    private double prevPitchDegs;

    private boolean reachedStation;
    private boolean passedCenter;
    private boolean balanced;

    /**
     * Constructs a default balancer command from a drive subsystem and a boolean indicating
     * whether to balance from the front or back.
     * 
     * @param driveSubsystem The drive subsystem.
     * @param reversed Whether to balance driving forwards or backwards. Defaults to backwards; pass `true` to balance forwards.
     */
    public DefaultBalancerCommand(BaseDrivetrain driveSubsystem) {
        super(driveSubsystem);

        drivePID = new PIDController(0.0072, 0.0, 0.0);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        System.out.println("------------------- Balancer initialized -------------------");
        reachedStation = false;
        passedCenter = false;
        balanced = false;
    }

    @Override
    public void execute() {
        double currentPitchDegs = ahrs.getPitch();

        if (!reachedStation) {
            returnDrivePower = -0.80;
            if (currentPitchDegs <= -15.0) {
                reachedStation = true;
                returnDrivePower = -0.17;
            }
        } else {
            double deltaPitchDegs = Math.abs(currentPitchDegs - prevPitchDegs); // calc magnitude of angular acceleration based on delta angle over time

            if (!passedCenter) {
                passedCenter = currentPitchDegs >= -5.0;
            } else {
                returnDrivePower = -1 * drivePID.calculate(currentPitchDegs, 0);
                // System.out.println(returnDrivePower);
                if (Math.abs(currentPitchDegs) <= 1.0 && deltaPitchDegs <= 0.1) balanced = true;
            }
        }

        // if (driveSubsystem instanceof BaseSwerveSubsystem) {
        //     ((BaseSwerveSubsystem) driveSubsystem).setDrivePowers(returnDrivePower, 0.0, 0.0, true);
        // } else {
             driveSubsystem.setDrivePowers(returnDrivePower);
        // }

        prevPitchDegs = currentPitchDegs;
    }

    @Override
    public boolean isFinished() {
        return reachedStation && balanced;
        // return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(0.20);
    }
}
