package frc.robot.positions;

import java.util.EnumSet;

import frc.robot.subsystems.tiltedelevator.ElevatorState;

public enum PlacePosition {
    C3_HYBRID(
        FieldPosition.C3,
        FieldPosition.C3_INIT,
        ElevatorState.GROUND
    ),
    C3_MID(
        FieldPosition.C3,
        FieldPosition.C3_INIT,
        ElevatorState.CONE_MID
    ),
    C3_HIGH(
        FieldPosition.C3,
        FieldPosition.C3_INIT,
        ElevatorState.CONE_HIGH
    ),
    C2_HYBRID(
        FieldPosition.C2,
        FieldPosition.C2_INIT,
        ElevatorState.GROUND
    ),
    C2_MID(
        FieldPosition.C2,
        FieldPosition.C2_INIT,
        ElevatorState.CUBE_MID
    ),
    C2_HIGH(
        FieldPosition.C2,
        FieldPosition.C2_INIT,
        ElevatorState.CUBE_HIGH
    ),
    C1_HYBRID(
        FieldPosition.C1,
        FieldPosition.C1_INIT,
        ElevatorState.GROUND
    ),
    C1_MID(
        FieldPosition.C1,
        FieldPosition.C1_INIT,
        ElevatorState.CONE_MID
    ),
    C1_HIGH(
        FieldPosition.C1,
        FieldPosition.C1_INIT,
        ElevatorState.CONE_HIGH
    ),

    B3_HYBRID(
        FieldPosition.B3,
        FieldPosition.B3_INIT,
        ElevatorState.GROUND
    ),
    B3_MID(
        FieldPosition.B3,
        FieldPosition.B3_INIT,
        ElevatorState.CONE_MID
    ),
    B3_HIGH(
        FieldPosition.B3,
        FieldPosition.B3_INIT,
        ElevatorState.CONE_HIGH
    ),
    B2_HYBRID(
        FieldPosition.B2,
        FieldPosition.B2_INIT,
        ElevatorState.GROUND
    ),
    B2_MID(
        FieldPosition.B2,
        FieldPosition.B2_INIT,
        ElevatorState.CUBE_MID
    ),
    B2_HIGH(
        FieldPosition.B2,
        FieldPosition.B2_INIT,
        ElevatorState.CUBE_HIGH
    ),
    B1_HYBRID(
        FieldPosition.B1,
        FieldPosition.B1_INIT,
        ElevatorState.GROUND
    ),
    B1_MID(
        FieldPosition.B1,
        FieldPosition.B1_INIT,
        ElevatorState.CONE_MID
    ),
    B1_HIGH(
        FieldPosition.B1,
        FieldPosition.B1_INIT,
        ElevatorState.CONE_HIGH
    ),

    A3_HYBRID(
        FieldPosition.A3,
        FieldPosition.A3_INIT,
        ElevatorState.GROUND
    ),
    A3_MID(
        FieldPosition.A3,
        FieldPosition.A3_INIT,
        ElevatorState.CONE_MID
    ),
    A3_HIGH(
        FieldPosition.A3,
        FieldPosition.A3_INIT,
        ElevatorState.CONE_HIGH
    ),
    A2_HYBRID(
        FieldPosition.A2,
        FieldPosition.A2_INIT,
        ElevatorState.GROUND
    ),
    A2_MID(
        FieldPosition.A2,
        FieldPosition.A2_INIT,
        ElevatorState.CUBE_MID
    ),
    A2_HIGH(
        FieldPosition.A2,
        FieldPosition.A2_INIT,
        ElevatorState.CUBE_HIGH
    ),
    A1_HYBRID(
        FieldPosition.A1,
        FieldPosition.A1_INIT,
        ElevatorState.GROUND
    ),
    A1_MID(
        FieldPosition.A1,
        FieldPosition.A1_INIT,
        ElevatorState.CONE_MID
    ),
    A1_HIGH(
        FieldPosition.A1,
        FieldPosition.A1_INIT,
        ElevatorState.CONE_HIGH
    );

    public final FieldPosition placePosition;
    public final FieldPosition alignPosition;
    public final ElevatorState elevatorState;

    /**
     * Constructs a `PlacePosition` from provided placing and aligning field positions and a provided elevator state.
     * 
     * @param placePosition The position of the robot when placing the piece.
     * @param alignPosition The position of the robot when aligning to the node.
     * @param elevatorState The elevator state.
     */
    private PlacePosition(FieldPosition placePosition, FieldPosition alignPosition, ElevatorState elevatorState) {
        this.placePosition = placePosition;
        this.alignPosition = alignPosition;
        this.elevatorState = elevatorState;
    }

    /**
     * Gets all `PlacePositions` that represent cube nodes.
     * @return An `EnumSet` of place positions that represent cube nodes.
     */
    public static EnumSet<PlacePosition> getCubePositions() {
        return EnumSet.of(
            PlacePosition.A2_HIGH, PlacePosition.A2_MID, PlacePosition.A2_HYBRID,
            PlacePosition.B2_HIGH, PlacePosition.B2_MID, PlacePosition.B2_HYBRID,
            PlacePosition.C2_HIGH, PlacePosition.C2_MID, PlacePosition.C2_HYBRID
        );
    }
}
