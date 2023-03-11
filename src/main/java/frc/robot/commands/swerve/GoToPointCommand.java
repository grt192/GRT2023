package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class GoToPointCommand extends CommandBase {
    private final BaseSwerveSubsystem swerveSubsystem;

    private final Pose2d targetPose;
    private Pose2d currentPose;

    private static final PIDController xController = new PIDController(1.5, 0, 0);
    private static final PIDController yController = new PIDController(1.5, 0, 0);

    private static final double X_TOLERANCE_METERS = Units.inchesToMeters(1.0);
    private static final double Y_TOLERANCE_METERS = Units.inchesToMeters(1.0);
    private static final double THETA_TOLERANCE_RADS = Math.toRadians(3.0);

    public GoToPointCommand(BaseSwerveSubsystem swerveSubsystem, Pose2d targetPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetPose = targetPose;
    }

    @Override
    public void execute() {
        currentPose = swerveSubsystem.getRobotPosition();

        // Calculate powers from PID, clamping them to within [-1.0, 1.0].
        double xPower = MathUtil.clamp(
            xController.calculate(currentPose.getX(), targetPose.getX()),
            -1.0, 1.0
        );
        double yPower = MathUtil.clamp(
            yController.calculate(currentPose.getY(), targetPose.getY()),
            -1.0, 1.0
        );

        // Pass powers and target heading to swerve subsystem
        swerveSubsystem.setDrivePowersWithHeadingLock(
            xPower, yPower, targetPose.getRotation(), false
        );
    }

    @Override
    public boolean isFinished() {
        double xErrorMeters = Math.abs(targetPose.getX() - currentPose.getX());
        double yErrorMeters = Math.abs(targetPose.getY() - currentPose.getY());
        double thetaErrorRads = Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());
        return xErrorMeters < X_TOLERANCE_METERS && yErrorMeters < Y_TOLERANCE_METERS && thetaErrorRads < THETA_TOLERANCE_RADS;
    }
}
