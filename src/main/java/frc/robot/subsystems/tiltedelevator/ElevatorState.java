package frc.robot.subsystems.tiltedelevator;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants;

public enum ElevatorState {
    GROUND(0, Units.inchesToMeters(4), 0),
    CHUTE(Units.inchesToMeters(40)),
    SUBSTATION(Units.inchesToMeters(45)), // absolute height = 37.375 in
    CUBE_MID(Units.inchesToMeters(Constants.IS_R1 ? 33 : 40)), // absolute height = 14.25 in
    CUBE_HIGH(Units.inchesToMeters(Constants.IS_R1 ? 53 : 55)), // absolute height = 31.625 in
    CONE_MID(Units.inchesToMeters(Constants.IS_R1 ? 50 : 48), 0, -Units.inchesToMeters(10)), // absolute height = 34 in
    CONE_HIGH(Units.inchesToMeters(Constants.IS_R1 ? 53 : 61)), // absolute height = 46 in
    HYBRID(Units.inchesToMeters(20)),//NEEDS TUNING
    HOME(Units.inchesToMeters(0));

    private final double extendDistanceMeters; // meters, extension distance of winch
    private final double withPieceOffset;
    private final double dropOffset;

    /**
     * ElevatorState defined by extension of elevator from zero. All values in
     * meters and radians.
     * 
     * @param extendDistanceMeters The extension of the subsystem.
     * @param withPieceOffset      The offset to apply to the extension distance
     *                             when the intake is holding a piece.
     * @param dropOffset           The offset to apply to the extension distance
     *                             when lowering the piece before dropping
     */
    private ElevatorState(double extendDistanceMeters, double withPieceOffset, double dropOffset) {
        this.extendDistanceMeters = extendDistanceMeters;
        this.withPieceOffset = withPieceOffset;
        this.dropOffset = dropOffset;
    }

    /**
     * ElevatorState defined by extension of elevator from zero. All values in meters and radians.
     * 
     * @param extendDistanceMeters The extension of the subsystem.
     */
    private ElevatorState(double extendDistanceMeters) {
        this(extendDistanceMeters, 0, 0);
    }

    public double getExtension(OffsetState offsetState, boolean hasPiece) {
        return switch (offsetState) {
            case ONLY_MANUAL_OFFSET -> this.extendDistanceMeters;
            case OVERRIDE_HAS_PIECE -> this.extendDistanceMeters + withPieceOffset;
            case DROPPING -> this.extendDistanceMeters + dropOffset;
            case DEFAULT -> {
                yield hasPiece ? this.extendDistanceMeters + withPieceOffset
                    : this.extendDistanceMeters;
            }
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
