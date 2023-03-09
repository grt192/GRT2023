package frc.robot.positions;

import frc.robot.subsystems.tiltedelevator.ElevatorState;

public enum PlacePosition {
    C3MID(
        FieldPosition.C3,
        FieldPosition.C3_INIT,
        ElevatorState.CONE_MID
    ),
    C3HIGH(
        FieldPosition.C3,
        FieldPosition.C3_INIT,
        ElevatorState.CONE_HIGH
    ),
    C2MID(
        FieldPosition.C2,
        FieldPosition.C2_INIT,
        ElevatorState.CUBE_MID
    ),
    C2HIGH(
        FieldPosition.C2,
        FieldPosition.C2_INIT,
        ElevatorState.CUBE_HIGH
    ),
    C1MID(
        FieldPosition.C1,
        FieldPosition.C1_INIT,
        ElevatorState.CONE_MID
    ),
    C1HIGH(
        FieldPosition.C1,
        FieldPosition.C1_INIT,
        ElevatorState.CONE_HIGH
    ),

    B3MID(
        FieldPosition.B3,
        FieldPosition.B3_INIT,
        ElevatorState.CONE_MID
    ),
    B3HIGH(
        FieldPosition.B3,
        FieldPosition.B3_INIT,
        ElevatorState.CONE_HIGH
    ),
    B2MID(
        FieldPosition.B2,
        FieldPosition.B2_INIT,
        ElevatorState.CUBE_MID
    ),
    B2HIGH(
        FieldPosition.B2,
        FieldPosition.B2_INIT,
        ElevatorState.CUBE_HIGH
    ),
    B1MID(
        FieldPosition.B1,
        FieldPosition.B1_INIT,
        ElevatorState.CONE_MID
    ),
    B1HIGH(
        FieldPosition.B1,
        FieldPosition.B1_INIT,
        ElevatorState.CONE_HIGH
    ),

    A3MID(
        FieldPosition.A3,
        FieldPosition.A3_INIT,
        ElevatorState.CONE_MID
    ),
    A3HIGH(
        FieldPosition.A3,
        FieldPosition.A3_INIT,
        ElevatorState.CONE_HIGH
    ),
    A2MID(
        FieldPosition.A2,
        FieldPosition.A2_INIT,
        ElevatorState.CUBE_MID
    ),
    A2HIGH(
        FieldPosition.A2,
        FieldPosition.A2_INIT,
        ElevatorState.CUBE_HIGH
    ),
    A1MID(
        FieldPosition.A1,
        FieldPosition.A1_INIT,
        ElevatorState.CONE_MID
    ),
    A1HIGH(
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
