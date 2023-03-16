package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class GoToPointCommand extends CommandBase {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final SwerveDriveKinematics kinematics;

    private final PIDController xController = new PIDController(3.5, 0, 0);
    private final PIDController yController = new PIDController(3.5, 0, 0);
    private final PIDController thetaController;

    private final SlewRateLimiter xRateLimiter;
    private final SlewRateLimiter yRateLimiter;
    private final SlewRateLimiter thetaRateLimiter;

    private final Pose2d targetPose;
    private Pose2d currentPose;

    private static final double X_TOLERANCE_METERS = Units.inchesToMeters(1.0);
    private static final double Y_TOLERANCE_METERS = Units.inchesToMeters(1.0);
    private static final double THETA_TOLERANCE_RADS = Math.toRadians(3.0);

    public GoToPointCommand(BaseSwerveSubsystem swerveSubsystem, Pose2d targetPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.kinematics = swerveSubsystem.getKinematics();
        this.thetaController = swerveSubsystem.getThetaController();

        this.xRateLimiter = new SlewRateLimiter(swerveSubsystem.MAX_ACCEL);
        this.yRateLimiter = new SlewRateLimiter(swerveSubsystem.MAX_ACCEL);
        this.thetaRateLimiter = new SlewRateLimiter(swerveSubsystem.MAX_ALPHA);

        this.targetPose = targetPose;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        currentPose = swerveSubsystem.getRobotPosition();

        // Calculate velocities from PID, clamping them to within [-MAX_VEL, MAX_VEL].
        double vx = MathUtil.clamp(
            xController.calculate(currentPose.getX(), targetPose.getX()),
            -swerveSubsystem.MAX_VEL, swerveSubsystem.MAX_VEL
        );
        double vy = MathUtil.clamp(
            yController.calculate(currentPose.getY(), targetPose.getY()),
            -swerveSubsystem.MAX_VEL, swerveSubsystem.MAX_VEL
        );
        double omega = MathUtil.clamp(
            thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()),
            -swerveSubsystem.MAX_OMEGA, swerveSubsystem.MAX_OMEGA
        );

        // Convert field-relative velocities into robot-relative chassis speeds.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xRateLimiter.calculate(vx),
            yRateLimiter.calculate(vy),
            thetaRateLimiter.calculate(omega),
            currentPose.getRotation()
        );

        // Calculate swerve module states from desired chassis speeds, desaturating
        // them to ensure all velocities are under MAX_VEL after kinematics.
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds, 
            swerveSubsystem.MAX_VEL, swerveSubsystem.MAX_VEL, swerveSubsystem.MAX_OMEGA
        );

        // Pass states to swerve subsystem
        swerveSubsystem.setSwerveModuleStates(states);
    }

    @Override
    public boolean isFinished() {
        double xErrorMeters = Math.abs(targetPose.getX() - currentPose.getX());
        double yErrorMeters = Math.abs(targetPose.getY() - currentPose.getY());
        double thetaErrorRads = Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());
        return xErrorMeters < X_TOLERANCE_METERS && yErrorMeters < Y_TOLERANCE_METERS && thetaErrorRads < THETA_TOLERANCE_RADS;
    }
}
