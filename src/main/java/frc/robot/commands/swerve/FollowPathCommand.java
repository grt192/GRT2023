package frc.robot.commands.swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class FollowPathCommand extends SwerveControllerCommand {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final Rotation2d targetAngle;

    private static final double xP = 0.4;
    private static final double xI = 0;
    private static final double xD = 0;

    private static final double yP = 0.4;
    private static final double yI = 0;
    private static final double yD = 0;

    private static final double thetaP = 1.5;
    private static final double thetaI = 0;
    private static final double thetaD = 0;

    private static final double ACCEPTABLE_ANGLE_ERROR_RADS = Math.toRadians(3.0);

    /**
     * Creates a FollowPathCommand from a given trajectory.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param trajectory The trajectory to follow.
     */
    public FollowPathCommand(BaseSwerveSubsystem swerveSubsystem, Trajectory trajectory) {
        super(
            trajectory,
            swerveSubsystem::getRobotPosition,
            swerveSubsystem.getKinematics(),
            new PIDController(xP, xI, xD),
            new PIDController(yP, yI, yD),
            new ProfiledPIDController(
                thetaP, thetaI, thetaD, 
                new TrapezoidProfile.Constraints(
                    swerveSubsystem.MAX_OMEGA,
                    swerveSubsystem.MAX_ALPHA
                )
            ),
            // () -> new Rotation2d(),
            swerveSubsystem::setSwerveModuleStates,
            swerveSubsystem
        );

        this.swerveSubsystem = swerveSubsystem;
        this.targetAngle = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();
    }

    /**
     * Creates a FollowPathCommand from a given start point, list of waypoints, end point, and boolean representing whether
     * the path should be reversed (if the robot should drive backwards through the trajectory).
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param start The start point of the trajectory as a Pose2d.
     * @param waypoints A list of waypoints the robot must pass through as a List<Translation2d>.
     * @param end The end point of the trajectory as a Pose2d.
     * @param reversed Whether the trajectory is reversed.
     */
    public FollowPathCommand(BaseSwerveSubsystem swerveSubsystem, Pose2d start, List<Translation2d> waypoints, Pose2d end, boolean reversed) {
        this(
            swerveSubsystem,
            // Target trajectory
            TrajectoryGenerator.generateTrajectory(
                start, waypoints, end, 
                new TrajectoryConfig(swerveSubsystem.MAX_VEL, swerveSubsystem.MAX_ACCEL)
                    .setReversed(reversed)
                    .setKinematics(swerveSubsystem.getKinematics())
                    .addConstraint(
                        new SwerveDriveKinematicsConstraint(
                            swerveSubsystem.getKinematics(), 
                            swerveSubsystem.MAX_VEL
                        )
                    )
            )
        );
    }

    /**
     * Creates a FollowPathCommand from a given start point, list of waypoints, and end point.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param start The start point of the trajectory as a Pose2d.
     * @param waypoints A list of waypoints the robot must pass through as a List<Translation2d>.
     * @param end The end point of the trajectory as a Pose2d.
     */
    public FollowPathCommand(BaseSwerveSubsystem swerveSubsystem, Pose2d start, List<Translation2d> waypoints, Pose2d end) {
        this(swerveSubsystem, start, waypoints, end, false);
    }

    @Override
    public boolean isFinished() {
        // End the command if the trajectory has finished and we are within tolerance of our final angle.
        // While the trajectory is finished but the robot is not at the target angle, the command will
        // continually pass the final (stationary) trajectory state and our target angle to the holonomic
        // controller to slowly bring the robot to the correct heading.
        double absAngleError = Math.abs(swerveSubsystem.getRobotPosition().getRotation().minus(targetAngle).getRadians());
        return super.isFinished() && absAngleError <= ACCEPTABLE_ANGLE_ERROR_RADS;
    }
}
