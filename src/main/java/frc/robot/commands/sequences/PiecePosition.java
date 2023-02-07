package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public interface PiecePosition {
    Pose2d getPose();

    public enum Red implements PiecePosition {
        PIECE1(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE2(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE3(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE4(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        ));

        private Pose2d pose;

        private Red(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }
    }

    public enum Blue implements PiecePosition {
        PIECE1(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE2(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE3(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE4(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        ));

        private Pose2d pose;

        private Blue(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }
    }
}
