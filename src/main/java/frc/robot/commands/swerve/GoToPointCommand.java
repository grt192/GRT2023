package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.util.TrackingTimer;

public class GoToPointCommand extends CommandBase {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final SwerveDriveKinematics kinematics;

    private final PIDController xController = new PIDController(3.5, 0, 0);
    private final PIDController yController = new PIDController(3.5, 0, 0);
    private final PIDController thetaController;

    private final Timer stopTimer = new Timer();

    private final Pose2d targetPose;
    private Pose2d currentPose;

    private static final double X_TOLERANCE_METERS = Units.inchesToMeters(1.0);
    private static final double Y_TOLERANCE_METERS = Units.inchesToMeters(1.0);
    private static final double THETA_TOLERANCE_RADS = Math.toRadians(3.0);
    private static final double SPEED_TOLERANCE_METERS_PER_SEC = 1;

    SwerveModulePosition[] previousModulePositions;

    public GoToPointCommand(BaseSwerveSubsystem swerveSubsystem, Pose2d targetPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.kinematics = swerveSubsystem.getKinematics();
        this.thetaController = swerveSubsystem.getThetaController();
        previousModulePositions = swerveSubsystem.getModuleStates();

        this.targetPose = targetPose;

        addRequirements(swerveSubsystem);

        stopTimer.start();
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
            vx, vy, omega,
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

    /**
     * Gets whether the robot is aligned to its target heading.
     * @param tolerance The tolerance, in radians, for the heading.
     * @return Whether the robot is aligned to its target heading.
     */
    public boolean isHeadingAligned(double tolerance) {
        // Semi-hacky solution to prevent null pointer exceptions from calling this
        // before a call of `.execute()`.
        if (currentPose == null) return false;

        double thetaErrorRads = Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());
        return thetaErrorRads < tolerance;
    }

    /**
     * Gets whether the robot is stopped or not
     * @param tolerance The tolerance, in meters/second, which the speed of EVERY swerve module must be under
     * @return Whether the robot is stopped or not
     */
    public boolean isStopped(double tolerance) {
        //iterates through each module, comparing its speed to the tolerance
        SwerveModulePosition[] modulePositions = swerveSubsystem.getModuleStates();

        for (int i = 0; i < 4; i++) {
            if(Math.abs(modulePositions[i].distanceMeters - previousModulePositions[i].distanceMeters) / stopTimer.get() > tolerance){
                return false;
            }
        }
        stopTimer.reset();

        System.out.println("Speed m/s " + modulePositions[0].distanceMeters);

        return true;
    }

    @Override
    public boolean isFinished() {
        double xErrorMeters = Math.abs(targetPose.getX() - currentPose.getX());
        double yErrorMeters = Math.abs(targetPose.getY() - currentPose.getY());
        isStopped(SPEED_TOLERANCE_METERS_PER_SEC);
        return xErrorMeters < X_TOLERANCE_METERS && yErrorMeters < Y_TOLERANCE_METERS && isHeadingAligned(THETA_TOLERANCE_RADS) && isStopped(SPEED_TOLERANCE_METERS_PER_SEC);
    }

    @Override
    public void end(boolean interrupted) {
        currentPose = null;
    }
}
