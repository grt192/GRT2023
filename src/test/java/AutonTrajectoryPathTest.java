import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import frc.robot.commands.sequences.BlueBalanceAuton;
import frc.robot.commands.sequences.BlueBottomAuton;
import frc.robot.commands.sequences.BlueTopAuton;
import frc.robot.commands.sequences.RedBalanceAuton;
import frc.robot.commands.sequences.RedBottomAuton;
import frc.robot.commands.sequences.RedTopAuton;
import frc.robot.commands.sequences.test.BoxAutonSequence;
import frc.robot.commands.sequences.test.GRTAutonSequence;
import frc.robot.commands.sequences.test.HighRotationLinePath;
import frc.robot.commands.sequences.test.RotatingSCurveAutonSequence;
import frc.robot.commands.sequences.test.StraightLinePath;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

import java.util.List;

public class AutonTrajectoryPathTest {
    private static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(null);
    private static final RollerSubsystem rollerSubsystem = new RollerSubsystem();
    private static final TiltedElevatorSubsystem tiltedElevatorSubsystem = new TiltedElevatorSubsystem();

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
                swerveSubsystem,
                new Pose2d(),
                List.of(),
                new Pose2d(1, 1, new Rotation2d())
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
                swerveSubsystem,
                new Pose2d(),
                List.of(new Translation2d(1, 1)),
                new Pose2d(2, 0, new Rotation2d())
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
                swerveSubsystem,
                new Pose2d(),
                List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
                ),
                new Pose2d(3, 0, new Rotation2d())
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
                swerveSubsystem,
                new Pose2d(),
                List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
                ),
                new Pose2d(3, 0, new Rotation2d()),
                new Rotation2d(Math.PI / 2.0),
                new Rotation2d(Math.PI / 2.0)
            ),
            Math.PI / 2.0,
            Math.PI / 2.0
        );
    }

    /**
     * Ensures that all test auton paths compile.
     */
    @Test
    public void compileTestPaths() {
        new StraightLinePath(swerveSubsystem);
        new HighRotationLinePath(swerveSubsystem);
        new RotatingSCurveAutonSequence(swerveSubsystem);
        new BoxAutonSequence(swerveSubsystem);
        new GRTAutonSequence(swerveSubsystem);
    }

    /**
     * Ensures that all red auton paths compile.
     */
    @Test
    public void compileRedPaths() {
        new RedTopAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        new RedBalanceAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        new RedBottomAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
    }

    /**
     * Ensures that all blue auton paths compile.
     */
    @Test
    public void compileBluePaths() {
        new BlueTopAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        new BlueBalanceAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
        new BlueBottomAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
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
