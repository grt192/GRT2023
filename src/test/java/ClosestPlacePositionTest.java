import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.commands.dropping.AutoAlignCommand;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.ElevatorState.OffsetState;

public class ClosestPlacePositionTest {
    /**
     * Ensures that the `getClosestPlacePosition` method correctly returns the closest place
     * position when the robot is at a given place position.
     * 
     * Current position: A1_INIT
     * Current elevator state: ElevatorState.HYBRID
     * Expected position: A1_HYBRID
     */
    @Test
    public void atHybrid() {
        runTest(
            PlacePosition.A1_HYBRID.alignPosition.BLUE,
            PlacePosition.A1_HYBRID.elevatorState,
            PlacePosition.A1_HYBRID
        );
    }

    /**
     * Ensures that the `getClosestPlacePosition` method correctly returns the closest place
     * position when the robot is at a given place position.
     * 
     * Current position: A1_INIT
     * Current elevator state: ElevatorState.CONE_MID
     * Expected position: A1_MID
     */
    @Test
    public void atConeMid() {
        runTest(
            PlacePosition.A1_MID.alignPosition.BLUE,
            PlacePosition.A1_MID.elevatorState,
            PlacePosition.A1_MID
        );
    }

    /**
     * Ensures that the `getClosestPlacePosition` method correctly returns the closest place
     * position when the robot is at a given place position.
     * 
     * Current position: A1_INIT
     * Current elevator state: ElevatorState.CONE_HIGH
     * Expected position: A1_HIGH
     */
    @Test
    public void atConeHigh() {
        runTest(
            PlacePosition.A1_HIGH.alignPosition.BLUE,
            PlacePosition.A1_HIGH.elevatorState,
            PlacePosition.A1_HIGH
        );
    }

    /**
     * Ensures that the `getClosestPlacePosition` method correctly returns the closest place
     * position when the robot is at a given place position.
     * 
     * Current position: A2_INIT
     * Current elevator state: ElevatorState.CUBE_MID
     * Expected position: A2_MID
     */
    @Test
    public void atCubeMid() {
        runTest(
            PlacePosition.A2_MID.alignPosition.BLUE,
            PlacePosition.A2_MID.elevatorState,
            PlacePosition.A2_MID
        );
    }

    /**
     * Ensures that the `getClosestPlacePosition` method correctly returns the closest place
     * position when the robot is at a given place position.
     * 
     * Current position: A2_INIT
     * Current elevator state: ElevatorState.CUBE_HIGH
     * Expected position: A2_HIGH
     */
    @Test
    public void atCubeHigh() {
        runTest(
            PlacePosition.A2_HIGH.alignPosition.BLUE,
            PlacePosition.A2_HIGH.elevatorState,
            PlacePosition.A2_HIGH
        );
    }

    /**
     * Ensures that the `getClosestPlacePosition` method correctly returns the closest place
     * position when the robot is near (10" from) a given place position.
     * 
     * Current position: A1_INIT + 10"
     * Current elevator state: ElevatorState.HYBRID
     * Expected position: A1_HYBRID
     */
    @Test
    public void nearHybrid() {
        runTest(
            PlacePosition.A1_HYBRID.alignPosition.BLUE.plus(
                new Transform2d(new Translation2d(Units.inchesToMeters(10), 0), new Rotation2d())
            ),
            PlacePosition.A1_HYBRID.elevatorState,
            PlacePosition.A1_HYBRID
        );
    }

    /**
     * Ensures that the `getClosestPlacePosition` method correctly returns the closest place
     * position when the robot is near (10" from) a given place position.
     * 
     * Current position: A1_INIT + 10"
     * Current elevator state: ElevatorState.CONE_HIGH
     * Expected position: A1_HIGH
     */
    @Test
    public void nearConeHigh() {
        runTest(
            PlacePosition.A1_HIGH.alignPosition.BLUE.plus(
                new Transform2d(new Translation2d(Units.inchesToMeters(10), 0), new Rotation2d())
            ),
            PlacePosition.A1_HIGH.elevatorState,
            PlacePosition.A1_HIGH
        );
    }

    /**
     * Ensures that the `getClosestPlacePosition` method correctly returns the closest place
     * position when the robot is near (10" from) a given place position.
     * 
     * Current position: A2_INIT + 10"
     * Current elevator state: ElevatorState.CUBE_HIGH
     * Expected position: A2_HIGH
     */
    @Test
    public void nearCubeHigh() {
        runTest(
            PlacePosition.A2_HIGH.alignPosition.BLUE.plus(
                new Transform2d(new Translation2d(Units.inchesToMeters(10), 0), new Rotation2d())
            ),
            PlacePosition.A2_HIGH.elevatorState,
            PlacePosition.A2_HIGH
        );
    }

    /**
     * Ensures that the `getClosestPlacePosition` method correctly breaks ties using elevator extension.
     * 
     * Current position: B2_INIT
     * Current elevator state: ElevatorState.CONE_HIGH (closer to CUBE_HIGH than CUBE_MID)
     * Expected position: B2_HIGH
     */
    @Test
    public void breaksElevatorTiesTowardsClosest() {
        runTest(
            PlacePosition.B2_HIGH.alignPosition.BLUE,
            ElevatorState.CONE_HIGH,
            PlacePosition.B2_HIGH
        );
    }

    /**
     * Ensures that the `getClosestPlacePosition` method correctly breaks ties using elevator extension.
     * 
     * Current position: between C1_INIT and C2_INIT
     * Current elevator state: ElevatorState.CONE_HIGH
     * Expected position: C1_HIGH
     */
    @Test
    public void breaksElevatorTiesTowardsCone() {
        Pose2d C1 = PlacePosition.C1_HIGH.alignPosition.BLUE;
        Pose2d C2 = PlacePosition.C2_HIGH.alignPosition.BLUE;

        runTest(
            new Pose2d(
                C1.getTranslation().plus(C2.getTranslation()).div(2),
                C1.getRotation()
            ),
            ElevatorState.CONE_HIGH,
            PlacePosition.C1_HIGH
        );
    }

    /**
     * Ensures that the `getClosestPlacePosition` method correctly breaks ties using elevator extension.
     * 
     * Current position: between C1_INIT and C2_INIT
     * Current elevator state: ElevatorState.CUBE_HIGH
     * Expected position: C2_HIGH
     */
    @Test
    public void breaksElevatorTiesTowardsCube() {
        Pose2d C1 = PlacePosition.C1_HIGH.alignPosition.BLUE;
        Pose2d C2 = PlacePosition.C2_HIGH.alignPosition.BLUE;

        runTest(
            new Pose2d(
                C1.getTranslation().plus(C2.getTranslation()).div(2),
                C1.getRotation()
            ),
            ElevatorState.CUBE_HIGH,
            PlacePosition.C2_HIGH
        );
    }

    /**
     * Runs a unit test with the given parameters.
     * @param robotPose The current pose of the robot.
     * @param elevatorState The current state of the elevator.
     * @param expectedPlacePosition The expected closest (blue) place position.
     */
    private void runTest(Pose2d robotPose, ElevatorState elevatorState, PlacePosition expectedPlacePosition) {
        double elevatorExtensionMeters = elevatorState.getExtension(OffsetState.DEFAULT, true);
        PlacePosition target = AutoAlignCommand.getClosestPlacePosition(robotPose, elevatorExtensionMeters, false);

        assertEquals(expectedPlacePosition, target);
    }
}
