package frc.robot.positions;

import frc.robot.subsystems.tiltedelevator.ElevatorState;

public enum PlacePosition {
    C3_HYBRID(
        FieldPosition.C3,
        FieldPosition.C3_INIT,
        ElevatorState.HYBRID
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
        ElevatorState.HYBRID
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
        ElevatorState.HYBRID
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
        ElevatorState.HYBRID
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
        ElevatorState.HYBRID
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
        ElevatorState.HYBRID
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
        ElevatorState.HYBRID
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
        ElevatorState.HYBRID
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
        ElevatorState.HYBRID
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
}
