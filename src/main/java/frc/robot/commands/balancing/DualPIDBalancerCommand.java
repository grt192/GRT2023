package frc.robot.commands.balancing;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

import static frc.robot.Constants.BalancerConstants.*;

public class DualPIDBalancerCommand extends BaseBalancerCommand {
    private final PIDController anglePID;
    private final PIDController deltaPID;

    private double returnDrivePower; // drive power to be returned to DT
    private double prevPitchDegs;

    private boolean reachedStation;
    private boolean balanced;

    public DualPIDBalancerCommand(BaseDrivetrain driveSubsystem) {
        super(driveSubsystem);

        anglePID = new PIDController(COMP_CHARGING_STATION_KP, 0.0, 0.0); 
        deltaPID = new PIDController(0.1 / 0.6, 0.0, 0.0); 
    }

    @Override
    public void initialize() {
        System.out.println("------------------- Balancer initialized -------------------");
        prevPitchDegs = ahrs.getPitch();
        reachedStation = false;
        balanced = false;
    }

    @Override
    public void execute() {
        double currentPitchDegs = ahrs.getPitch();

        if (!reachedStation) {
            returnDrivePower = -0.80;
            reachedStation = currentPitchDegs <= -7.0;
        } else {
            double deltaPitchDegs = Math.abs(currentPitchDegs - prevPitchDegs);
            // System.out.println("DeltaAngle: " + deltaPitchDegs);

            returnDrivePower = -1 * anglePID.calculate(currentPitchDegs, 0) + -1 * deltaPID.calculate(deltaPitchDegs, 0);

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
