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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class FollowPathCommand extends SwerveControllerCommand {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final Rotation2d targetAngle;

    private static final double xP = 2.5;
    private static final double xI = 0;
    private static final double xD = 0;

    private static final double yP = 2.5;
    private static final double yI = 0;
    private static final double yD = 0;

    private static final double thetaP = 1.5;
    private static final double thetaI = 0;
    private static final double thetaD = 0;

    private static final double ACCEPTABLE_ANGLE_ERROR_RADS = Math.toRadians(3.0);

    /**
     * Creates a FollowPathCommand from a given trajectory and target robot angle.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param trajectory The trajectory to follow.
     * @param targetAngle The target angle of the robot.
     */
    public FollowPathCommand(BaseSwerveSubsystem swerveSubsystem, Trajectory trajectory, Rotation2d targetAngle) {
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
            () -> targetAngle,
            swerveSubsystem::setSwerveModuleStates,
            swerveSubsystem
        );

        this.swerveSubsystem = swerveSubsystem;
        this.targetAngle = targetAngle;
    }

    /**
     * Creates a FollowPathCommand from a given start point, list of waypoints, and end point. The wheel headings at the start
     * and end are automatically generated using a straight-line to the closest waypoint.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param start The start point of the trajectory as a Pose2d.
     * @param waypoints A list of waypoints the robot must pass through as a List<Translation2d>.
     * @param end The end point of the trajectory as a Pose2d.
     * @return The created `FollowPathCommand`.
     */
    public static FollowPathCommand from(BaseSwerveSubsystem swerveSubsystem, Pose2d start, List<Translation2d> waypoints, Pose2d end) {
        return new FollowPathCommand(
            swerveSubsystem,
            createWheelHeadingTrajectory(
                start, waypoints, end,
                createDefaultConfig(swerveSubsystem)
            ),
            end.getRotation()
        );
    }

    /**
     * Creates a FollowPathCommand from a given start point, list of waypoints, and end point. The wheel headings at the start
     * and end are automatically generated using a straight-line to the closest waypoint.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param start The start point of the trajectory as a Pose2d.
     * @param waypoints A list of waypoints the robot must pass through as a List<Translation2d>.
     * @param end The end point of the trajectory as a Pose2d.
     * @param startsMoving Whether the trajectory should start in motion.
     * @param endsMoving Whether the trajectory should end in motion.
     * @return The created `FollowPathCommand`.
     */
    public static FollowPathCommand from(
        BaseSwerveSubsystem swerveSubsystem, Pose2d start, List<Translation2d> waypoints, Pose2d end,
        boolean startsMoving, boolean endsMoving
    ) {
        return new FollowPathCommand(
            swerveSubsystem,
            createWheelHeadingTrajectory(
                start, waypoints, end,
                createDefaultConfig(swerveSubsystem)
                    .setStartVelocity(startsMoving ? swerveSubsystem.MAX_VEL : 0.0)
                    .setEndVelocity(endsMoving ? swerveSubsystem.MAX_VEL : 0.0)
            ),
            end.getRotation()
        );
    }

    /**
     * Composes a sequence of FollowPathCommands from a given start point, list of waypoints, and end point. The commands
     * are composed such that the robot starts and ends at rest, but does not stop in the middle between commands. The robot
     * will straight-line between waypoints, hitting each heading along the way.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param start The start point of the trajectory as a Pose2d.
     * @param waypoints A list of waypoints the robot must pass through as a List<Pose2d>.
     * @param end The end point of the trajectory as a Pose2d.
     * @return The created `Command`.
     */
    public static Command composedFrom(BaseSwerveSubsystem swerveSubsystem, Pose2d start, List<Pose2d> waypoints, Pose2d end) {
        return composedFrom(swerveSubsystem, start, waypoints, end, false, false);
    }

    /**
     * Composes a sequence of FollowPathCommands from a given start point, list of waypoints, and end point. The commands
     * are composed such that the robot starts and ends moving according to the specified booleans, but does not stop in the
     * middle between commands. The robot ill straight-line between waypoints, hitting each heading along the way.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param start The start point of the trajectory as a Pose2d.
     * @param waypoints A list of waypoints the robot must pass through as a List<Pose2d>.
     * @param end The end point of the trajectory as a Pose2d.
     * @param startsMoving Whether the trajectory should start in motion.
     * @param endsMoving Whether the trajectory should end in motion.
     * @return The created `Command`.
     */
    public static Command composedFrom(
        BaseSwerveSubsystem swerveSubsystem, Pose2d start, List<Pose2d> waypoints, Pose2d end,
        boolean startsMoving, boolean endsMoving
    ) {
        if (waypoints.size() == 0) return from(swerveSubsystem, start, List.of(), end);

        Command sequence = from(swerveSubsystem, start, List.of(), waypoints.get(0), startsMoving, true);
        for (int i = 0; i < waypoints.size() - 1; i++) {
            sequence = sequence.andThen(from(
                swerveSubsystem, waypoints.get(i), List.of(), waypoints.get(i + 1),
                true, true
            ));
        }

        return sequence.andThen(from(
            swerveSubsystem, waypoints.get(waypoints.size() - 1), List.of(), end,
            true, endsMoving
        ));
    }

    /**
     * Creates a FollowPathCommand from a given start point, list of waypoints, end point, and wheel headings at
     * the start and end of the path.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param start The start point of the trajectory as a Pose2d.
     * @param waypoints A list of waypoints the robot must pass through as a List<Translation2d>.
     * @param end The end point of the trajectory as a Pose2d.
     * @param startHeading The wheel heading at the start of the path.
     * @param endHeading The wheel heading at the end of the path.
     * @return The created `FollowPathCommand`.
     */
    public static FollowPathCommand fromWheelHeadings(
        BaseSwerveSubsystem swerveSubsystem, Pose2d start, List<Translation2d> waypoints, Pose2d end,
        Rotation2d startHeading, Rotation2d endHeading
    ) {
        return new FollowPathCommand(
            swerveSubsystem,
            createWheelHeadingTrajectory(
                start, waypoints, end,
                startHeading, endHeading,
                createDefaultConfig(swerveSubsystem)
            ),
            end.getRotation()
        );
    }

    /**
     * Gets the wheel headings between two points as a `Rotation2d`.
     * 
     * @param a The start point of the segment.
     * @param b The end point of the segment.
     * @return The `Rotation2d` representing the wheel headings from a to b.
     */
    private static Rotation2d wheelHeadingFromPoints(Translation2d a, Translation2d b) {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();

        return new Rotation2d(dx, dy);
    }

    /**
     * Creates a Trajectory from a given start point, list of waypoints, and end point. The wheel headings at the start
     * and end are automatically generated using a straight-line to the closest waypoint.
     * 
     * @param start The start point of the trajectory as a Pose2d.
     * @param waypoints A list of waypoints the robot must pass through as a List<Translation2d>.
     * @param end The end point of the trajectory as a Pose2d.
     * @param config The trajectory config to use.
     * @return The created `Trajectory`.
     */
    public static Trajectory createWheelHeadingTrajectory(
        Pose2d start, List<Translation2d> waypoints, Pose2d end,
        TrajectoryConfig config
    ) {
        Translation2d startWaypoint = waypoints.size() > 0 
            ? waypoints.get(0) : end.getTranslation();
        Translation2d endWaypoint = waypoints.size() > 0 
            ? waypoints.get(waypoints.size() - 1) : start.getTranslation();

        return createWheelHeadingTrajectory(
            start, waypoints, end,
            wheelHeadingFromPoints(start.getTranslation(), startWaypoint),
            wheelHeadingFromPoints(endWaypoint, end.getTranslation()),
            config
        );
    }

    /**
     * Creates a Trajectory from a given start point, list of waypoints, end point, and wheel headings at
     * the start and end of the path.
     * 
     * @param start The start point of the trajectory as a Pose2d.
     * @param waypoints A list of waypoints the robot must pass through as a List<Translation2d>.
     * @param end The end point of the trajectory as a Pose2d.
     * @param startHeading The wheel heading at the start of the path.
     * @param endHeading The wheel heading at the end of the path.
     * @param config The trajectory config to use.
     * @return The created `Trajectory`.
     */
    public static Trajectory createWheelHeadingTrajectory(
        Pose2d start, List<Translation2d> waypoints, Pose2d end,
        Rotation2d startHeading, Rotation2d endHeading, TrajectoryConfig config
    ) {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(start.getTranslation(), startHeading),
            waypoints,
            new Pose2d(end.getTranslation(), endHeading), 
            config
        );
    }

    /**
     * Creates a default `TrajectoryConfig` from a `BaseSwerveSubsystem`'s constraints.
     * @param swerveSubsystem The `BaseSwerveSubsystem` to create a trajectory for.
     * @return The generated `TrajectoryConfig`.
     */
    private static TrajectoryConfig createDefaultConfig(BaseSwerveSubsystem swerveSubsystem) {
        return new TrajectoryConfig(swerveSubsystem.MAX_VEL, swerveSubsystem.MAX_ACCEL)
            .setKinematics(swerveSubsystem.getKinematics())
            .addConstraint(
                new SwerveDriveKinematicsConstraint(
                    swerveSubsystem.getKinematics(), 
                    swerveSubsystem.MAX_VEL
                )
            );
    }

    /*
    @Override
    public boolean isFinished() {
        // End the command if the trajectory has finished and we are within tolerance of our final angle.
        // While the trajectory is finished but the robot is not at the target angle, the command will
        // continually pass the final (stationary) trajectory state and our target angle to the holonomic
        // controller to slowly bring the robot to the correct heading.
        double absAngleError = Math.abs(swerveSubsystem.getRobotPosition().getRotation().minus(targetAngle).getRadians());
        return super.isFinished() && absAngleError <= ACCEPTABLE_ANGLE_ERROR_RADS;
    }
    */
}
