package frc.robot.subsystems.tiltedelevator;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants;

public enum ElevatorState {
    GROUND(0, Units.inchesToMeters(5), 0),
    CHUTE(Units.inchesToMeters(40)),
    SUBSTATION(Units.inchesToMeters(45)), // absolute height = 37.375 in
    CUBE_MID(Units.inchesToMeters(Constants.IS_R1 ? 33 : 40)), // absolute height = 14.25 in
    CUBE_HIGH(Units.inchesToMeters(Constants.IS_R1 ? 53 : 55)), // absolute height = 31.625 in
    CONE_MID(Units.inchesToMeters(Constants.IS_R1 ? 50 : 48), 0, -Units.inchesToMeters(10)), // absolute height = 34 in
    CONE_HIGH(Units.inchesToMeters(Constants.IS_R1 ? 53 : 61)), // absolute height = 46 in
    HYBRID(Units.inchesToMeters(20)),//NEEDS TUNING
    HOME(Units.inchesToMeters(0));

    private final double extendDistanceMeters; // meters, extension distance of winch
    private final double pieceOffsetMeters;
    private final double dropOffsetMeters;

    /**
     * Creates an `ElevatorState` from the extension of the elevator from zero, the state's offset
     * when there is a piece in the subsystem, and the state's offset when dropping.
     * 
     * @param extendDistanceMeters The extension of the subsystem.
     * @param pieceOffsetMeters The offset to apply to the extension distance when the intake is holding a piece.
     * @param dropOffsetMeters The offset to apply to the extension distance when lowering the piece before dropping.
     */
    private ElevatorState(double extendDistanceMeters, double pieceOffsetMeters, double dropOffsetMeters) {
        this.extendDistanceMeters = extendDistanceMeters;
        this.pieceOffsetMeters = pieceOffsetMeters;
        this.dropOffsetMeters = dropOffsetMeters;
    }

    /**
     * Creates an `ElevatorState` from the extension of the elevator from zero
     * @param extendDistanceMeters The extension of the subsystem.
     */
    private ElevatorState(double extendDistanceMeters) {
        this(extendDistanceMeters, 0, 0);
    }

    /**
     * Gets the extension commanded by this elevator state, given the `OffsetState` of 
     * the subsystem and whether there is currently a piece in the rollers.
     * 
     * @param offsetState The offset state of the subsystem.
     * @param hasPiece Whether there is a piece in the subsystem.
     * @return The extension, in meters, commanded by the state.
     */
    public double getExtension(OffsetState offsetState, boolean hasPiece) {
        return switch (offsetState) {
            case ONLY_MANUAL_OFFSET -> this.extendDistanceMeters;
            case OVERRIDE_HAS_PIECE -> this.extendDistanceMeters + pieceOffsetMeters;
            case DROPPING -> this.extendDistanceMeters + dropOffsetMeters;
            case DEFAULT -> hasPiece 
                ? this.extendDistanceMeters + pieceOffsetMeters
                : this.extendDistanceMeters;
        };
    }

    public enum OffsetState {
        /**
         * Use the current extension state's getWithPieceExtension and apply
         * offsetDistMeters.
         */
        OVERRIDE_HAS_PIECE,
        /**
         * Use the current extension state's getDropExtension and apply offsetDistMeters.
         */
        DROPPING,
        /**
         * Use the current extension state's getExtension and apply offsetDistMeters.
         */
        ONLY_MANUAL_OFFSET,
        /**
         * Use the current extension state's getExtension, or automatically use
         * getWithPieceExtension if hasPiece is true, and apply offsetDistMeters.
         */
        DEFAULT
    }
}
