package frc.robot.positions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;

public enum PlacePosition {
    C3MID(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(195.55),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CONE_MID),
    C3HIGH(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(195.55),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CONE_HIGH),
    C2MID(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(174.123),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CUBE_MID),
    C2HIGH(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(174.123),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CUBE_HIGH),
    C1MID(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(152.123),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CONE_MID),
    C1HIGH(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(152.123),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CONE_HIGH),
    B3MID(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(129.75),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CONE_MID),
    B3HIGH(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(129.75),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CONE_HIGH),
    B2MID(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(107.801),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CUBE_MID),
    B2HIGH(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(107.801),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CUBE_HIGH),
    B1MID(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(86.149),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CUBE_MID),
    B1HIGH(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(86.149),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CUBE_HIGH),
    A3MID(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(64.818),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CONE_MID),
    A3HIGH(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(64.818),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CONE_HIGH),
    A2MID(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(41.761),
        Rotation2d.fromDegrees(180)
    ),  ElevatorState.CUBE_MID),
    A2HIGH(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(41.761),
        Rotation2d.fromDegrees(180)
    ),  ElevatorState.CUBE_HIGH),
    A1MID(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(20.016),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CONE_MID), 
    A1HIGH(new Pose2d(
        Units.inchesToMeters(77.406),
        Units.inchesToMeters(20.016),
        Rotation2d.fromDegrees(180)
    ), ElevatorState.CONE_HIGH);

    public static double BLUE_INIT_X_IN = 75.178;
    public static double RED_INIT_X_IN = 573.895;

    public final PlaceState RED;
    public final PlaceState BLUE;

    private PlacePosition(Pose2d pose, ElevatorState elevatorState) {
        // TODO: mirror
        this.RED = new PlaceState(pose, elevatorState);
        this.BLUE = new PlaceState(pose, elevatorState);
    }

    public class PlaceState {
        private Pose2d pose;
        private ElevatorState elevatorState;

        public PlaceState(Pose2d pose, ElevatorState elevatorState) {
            this.pose = pose;
            this.elevatorState = elevatorState;
        }
    
        public Pose2d getPose() {
            return pose;
        }
    
        public ElevatorState getElevatorState() {
            return elevatorState;
        }
    }
}
