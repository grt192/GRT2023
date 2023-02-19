import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;

import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

import java.util.List;

public class WheelHeadingTrajectoryTest {
    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        SwerveConstants.TL_POS,
        SwerveConstants.TR_POS,
        SwerveConstants.BL_POS,
        SwerveConstants.BR_POS
    );
    private static final TrajectoryConfig config = new TrajectoryConfig(SwerveSubsystem.MAX_VEL, SwerveSubsystem.MAX_ACCEL)
        .setKinematics(kinematics)
        .addConstraint(new SwerveDriveKinematicsConstraint(kinematics, SwerveSubsystem.MAX_VEL));

    private static final double ACCEPTABLE_ANGLE_DELTA = 1e-15;

    /**
     * Ensures that wheel headings are correctly generated with a no-waypoint path.
     * Start: (0, 0) -> pi/4 to end
     * End: (1, 1)
     */
    @Test
    public void trajectoryNoWaypoint() {
        runTrajectoryTest(
            FollowPathCommand.createWheelHeadingTrajectory(
                new Pose2d(),
                List.of(),
                new Pose2d(1, 1, new Rotation2d()),
                config
            ),
            Math.PI / 4.0,
            Math.PI / 4.0
        );
    }

    /**
     * Ensures that wheel headings are correctly generated with a one-waypoint path.
     * Start: (0, 0) -> pi/4 to waypoint
     * Waypoint: (1, 1) -> -pi/4 to end
     * End: (2, 0)
     */
    @Test
    public void trajectoryOneWaypoint() {
        runTrajectoryTest(
            FollowPathCommand.createWheelHeadingTrajectory(
                new Pose2d(),
                List.of(new Translation2d(1, 1)),
                new Pose2d(2, 0, new Rotation2d()),
                config
            ),
            Math.PI / 4.0,
            -Math.PI / 4.0
        );
    }

    /**
     * Ensures that wheel headings are correctly generated with a two-waypoint path.
     * Start: (0, 0) -> pi/4 to waypoint 1
     * Waypoint 1: (1, 1)
     * Waypoint 2: (2, -1) -> pi/4 to end
     * End: (3, 0)
     */
    @Test
    public void trajectoryTwoWaypoints() {
        runTrajectoryTest(
            FollowPathCommand.createWheelHeadingTrajectory(
                new Pose2d(),
                List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
                ),
                new Pose2d(3, 0, new Rotation2d()),
                config
            ),
            Math.PI / 4.0,
            Math.PI / 4.0
        );
    }

    /**
     * Ensures that wheel headings are correctly generated when custom headings are passed in.
     */
    @Test
    public void trajectoryCustomHeadings() {
        runTrajectoryTest(
            FollowPathCommand.createWheelHeadingTrajectory(
                new Pose2d(),
                List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
                ),
                new Pose2d(3, 0, new Rotation2d()),
                new Rotation2d(Math.PI / 2.0),
                new Rotation2d(Math.PI / 2.0),
                config
            ),
            Math.PI / 2.0,
            Math.PI / 2.0
        );
    }

    /**
     * Tests a `FollowPathCommand` wheel-heading trajectory, asserting that it must end at rest (0.0 velocity)
     * and the start and end wheel headings must match given values.
     * 
     * @param trajectory The trajectory to test.
     * @param expectedStartHeadingRads The expected start wheel heading of the trajectory, in radians.
     * @param expectedEndHeadingRads The expected end wheel heading of the trajectory, in radians.
     */
    private static void runTrajectoryTest(
        Trajectory trajectory, double expectedStartHeadingRads, double expectedEndHeadingRads
    ) {
        List<Trajectory.State> states = trajectory.getStates();
        Trajectory.State firstState = states.get(0);
        Trajectory.State lastState = states.get(states.size() - 1);

        assertEquals(firstState.poseMeters.getRotation().getRadians(), expectedStartHeadingRads, ACCEPTABLE_ANGLE_DELTA);
        assertEquals(lastState.poseMeters.getRotation().getRadians(), expectedEndHeadingRads, ACCEPTABLE_ANGLE_DELTA);
        assertEquals(lastState.velocityMetersPerSecond, 0.0);
    }
}
