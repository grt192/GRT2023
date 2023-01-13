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

import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class FollowPathCommand extends SwerveControllerCommand {
    // TODO: tune / measure
    private static final double Kp = 3.3283;
    private static final double Ki = 0;
    private static final double Kd = 0;

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
            new PIDController(Kp, Ki, Kd),
            new PIDController(Kp, Ki, Kd),
            new ProfiledPIDController(
                Kp, Ki, Kd, 
                new TrapezoidProfile.Constraints(SwerveSubsystem.MAX_VEL, SwerveSubsystem.MAX_ACCEL)
            ),
            () -> new Rotation2d(), // TODO: this can control the angle of swerve at every timestep; hub locking?
            swerveSubsystem::setSwerveModuleStates,
            swerveSubsystem
        );
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
                new TrajectoryConfig(SwerveSubsystem.MAX_VEL, SwerveSubsystem.MAX_ACCEL)
                    .setReversed(reversed)
                    .setKinematics(swerveSubsystem.getKinematics())
                    .addConstraint(
                        new SwerveDriveKinematicsConstraint(
                            swerveSubsystem.getKinematics(), 
                            SwerveSubsystem.MAX_VEL
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
}
