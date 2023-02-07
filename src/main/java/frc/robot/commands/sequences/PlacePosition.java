package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;

public interface PlacePosition {
    Pose2d getPose();
    ElevatorState getElevatorState();

    double BLUE_INIT_X_IN = 70.007;
    double BLUE_PLACE_X_IN = 72.013;

    double RED_INIT_X_IN = 581.072;
    double RED_PLACE_X_IN = 578.737;

    public enum Red implements PlacePosition {
        C3MID(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(195.55),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CONE_MID),
        C3HIGH(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(195.55),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CONE_HIGH),
        C2MID(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(174.123),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CUBE_MID),
        C2HIGH(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(174.123),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CUBE_HIGH),
        C1MID(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(152.123),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CONE_MID),
        C1HIGH(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(152.123),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CONE_HIGH),
        B3MID(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(129.75),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CONE_MID),
        B3HIGH(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(129.75),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CONE_HIGH),
        B2MID(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(107.801),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CUBE_MID),
        B2HIGH(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(107.801),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CUBE_HIGH),
        B1MID(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(86.149),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CONE_MID),
        B1HIGH(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(86.149),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CONE_HIGH),
        A3MID(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(64.818),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CONE_MID),
        A3HIGH(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(64.818),
            Rotation2d.fromDegrees(0)
        ), ElevatorState.CONE_HIGH); /*,
        X8(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(41.761),
            Rotation2d.fromDegrees(0)
        )),
        X9(new Pose2d(
            Units.inchesToMeters(RED_PLACE_X_IN),
            Units.inchesToMeters(20.016),
            Rotation2d.fromDegrees(0)
        ));
        */

        private Pose2d pose;
        private ElevatorState elevatorState;

        private Red(Pose2d pose, ElevatorState elevatorState) {
            this.pose = pose;
            this.elevatorState = elevatorState;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public ElevatorState getElevatorState() {
            return elevatorState;
        }
    }

    public enum Blue implements PlacePosition {
        PLACEHOLDER(new Pose2d(
            Units.inchesToMeters(BLUE_PLACE_X_IN),
            Units.inchesToMeters(195.55),
            Rotation2d.fromDegrees(180)
        ), ElevatorState.GROUND);
        /*
        X1(new Pose2d(
            Units.inchesToMeters(BLUE_PLACE_X_IN),
            Units.inchesToMeters(195.55),
            Rotation2d.fromDegrees(180)
        )),
        X2(new Pose2d(
            Units.inchesToMeters(BLUE_PLACE_X_IN),
            Units.inchesToMeters(174.123),
            Rotation2d.fromDegrees(180)
        )),
        X3(new Pose2d(
            Units.inchesToMeters(BLUE_PLACE_X_IN),
            Units.inchesToMeters(152.123),
            Rotation2d.fromDegrees(180)
        )),
        X4(new Pose2d(
            Units.inchesToMeters(BLUE_PLACE_X_IN),
            Units.inchesToMeters(129.75),
            Rotation2d.fromDegrees(180)
        )),
        X5(new Pose2d(
            Units.inchesToMeters(BLUE_PLACE_X_IN),
            Units.inchesToMeters(107.801),
            Rotation2d.fromDegrees(180)
        )),
        X6(new Pose2d(
            Units.inchesToMeters(BLUE_PLACE_X_IN),
            Units.inchesToMeters(86.149),
            Rotation2d.fromDegrees(180)
        )),
        X7(new Pose2d(
            Units.inchesToMeters(BLUE_PLACE_X_IN),
            Units.inchesToMeters(64.818),
            Rotation2d.fromDegrees(180)
        )),
        X8(new Pose2d(
            Units.inchesToMeters(BLUE_PLACE_X_IN),
            Units.inchesToMeters(41.761),
            Rotation2d.fromDegrees(180)
        )),
        X9(new Pose2d(
            Units.inchesToMeters(BLUE_PLACE_X_IN),
            Units.inchesToMeters(20.016),
            Rotation2d.fromDegrees(180)
        ));
        */

        private Pose2d pose;
        private ElevatorState elevatorState;

        private Blue(Pose2d pose, ElevatorState elevatorState) {
            this.pose = pose;
            this.elevatorState = elevatorState;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public ElevatorState getElevatorState() {
            return elevatorState;
        }
    }
}
