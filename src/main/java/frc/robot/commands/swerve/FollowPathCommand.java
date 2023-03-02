package frc.robot.commands.swerve;

import java.util.ArrayList;
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
    private static final double xP = 1.5;
    private static final double xI = 0;
    private static final double xD = 0;

    private static final double yP = 1.5;
    private static final double yI = 0;
    private static final double yD = 0;

    private static final double thetaP = 1.5;
    private static final double thetaI = 0;
    private static final double thetaD = 0;

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

        // Set swerve subsystem target heading for teleop
        swerveSubsystem.setTargetHeading(targetAngle);
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
        return from(swerveSubsystem, start, waypoints, end, false, false);
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
                createConfig(swerveSubsystem, startsMoving, endsMoving)
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

        // List of remaining poses to hit, including the end pose
        ArrayList<Pose2d> poses = new ArrayList<>(waypoints);
        poses.add(end);

        // Start the path with the normal straight-line wheel-headings, but make each intermediate
        // waypoint's heading the average of the previous and next headings.
        Rotation2d currentWheelHeading = wheelHeadingFromPoints(start.getTranslation(), poses.get(0).getTranslation());
        Rotation2d nextWheelHeading = wheelHeadingFromPoints(poses.get(0).getTranslation(), poses.get(1).getTranslation());

        Rotation2d startHeading = currentWheelHeading;
        Rotation2d endHeading = averageWheelHeadings(currentWheelHeading, nextWheelHeading);

        SequentialCommandGroup sequence = new SequentialCommandGroup(fromWheelHeadings(
            swerveSubsystem, start, List.of(), poses.get(0),
            startHeading, endHeading, startsMoving, true
        ));

        for (int i = 0; i < poses.size() - 2; i++) {
            // For intermediate segments, the start heading is the previous end heading.
            // The end heading is still the average of the current and next wheel headings.
            currentWheelHeading = nextWheelHeading;
            nextWheelHeading = wheelHeadingFromPoints(poses.get(i + 1).getTranslation(), poses.get(i + 2).getTranslation());

            startHeading = endHeading;
            endHeading = averageWheelHeadings(currentWheelHeading, nextWheelHeading);

            sequence.addCommands(fromWheelHeadings(
                swerveSubsystem, poses.get(i), List.of(), poses.get(i + 1),
                startHeading, endHeading, true, true
            ));
        }

        sequence.addCommands(fromWheelHeadings(
            swerveSubsystem, poses.get(poses.size() - 2), List.of(), end,
            endHeading, wheelHeadingFromPoints(poses.get(poses.size() - 2).getTranslation(), end.getTranslation()),
            true, endsMoving
        ));
        return sequence;
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
        return fromWheelHeadings(swerveSubsystem, start, waypoints, end, startHeading, endHeading, false, false);
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
     * @param startsMoving Whether the trajectory should start in motion.
     * @param endsMoving Whether the trajectory should end in motion.
     * @return The created `FollowPathCommand`.
     */
    public static FollowPathCommand fromWheelHeadings(
        BaseSwerveSubsystem swerveSubsystem, Pose2d start, List<Translation2d> waypoints, Pose2d end,
        Rotation2d startHeading, Rotation2d endHeading, boolean startsMoving, boolean endsMoving
    ) {
        return new FollowPathCommand(
            swerveSubsystem,
            createWheelHeadingTrajectory(
                start, waypoints, end,
                startHeading, endHeading,
                createConfig(swerveSubsystem, startsMoving, endsMoving)
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
     * Gets the average angle between two wheel headings, accounting for angle wraparound at 180 degrees.
     * 
     * @param headingA The first wheel heading.
     * @param headingB The second wheel heading.
     * @return The `Rotation2d` representing the angle between A and B.
     */
    public static Rotation2d averageWheelHeadings(Rotation2d headingA, Rotation2d headingB) {
        if (Math.abs(headingA.getRadians() - headingB.getRadians()) < Math.PI)
            return headingA.plus(headingB).div(2.0);

        return headingA.plus(headingB).plus(Rotation2d.fromDegrees(180)).div(2.0);
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
     * Creates a `TrajectoryConfig` from a `BaseSwerveSubsystem`'s constraints and provided starts- and
     * ends-moving parameters.
     * 
     * @param swerveSubsystem The `BaseSwerveSubsystem` to create a trajectory for.
     * @param startsMoving Whether the trajectory should start in motion.
     * @param endsMoving Whether the trajectory should end in motion.
     * @return The generated `TrajectoryConfig`.
     */
    private static TrajectoryConfig createConfig(
        BaseSwerveSubsystem swerveSubsystem, boolean startsMoving, boolean endsMoving
    ) {
        return new TrajectoryConfig(swerveSubsystem.MAX_VEL, swerveSubsystem.MAX_ACCEL)
            .setKinematics(swerveSubsystem.getKinematics())
            .setStartVelocity(startsMoving ? swerveSubsystem.MAX_VEL : 0.0)
            .setEndVelocity(endsMoving ? swerveSubsystem.MAX_VEL : 0.0)
            .addConstraint(
                new SwerveDriveKinematicsConstraint(
                    swerveSubsystem.getKinematics(), 
                    swerveSubsystem.MAX_VEL
                )
            );
    }
}
