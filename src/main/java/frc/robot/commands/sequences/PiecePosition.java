package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public interface PiecePosition {
    double blueX = 278.264;
    double redX = 372.486;

    Pose2d getPose();

    public enum Blue implements PiecePosition {
        PIECE1(new Pose2d(
            Units.inchesToMeters(blueX),
            Units.inchesToMeters(179.893),
            Rotation2d.fromDegrees(0)
        )),
        PIECE2(new Pose2d(
            Units.inchesToMeters(blueX),
            Units.inchesToMeters(131.829),
            Rotation2d.fromDegrees(0)
        )),
        PIECE3(new Pose2d(
            Units.inchesToMeters(blueX),
            Units.inchesToMeters(83.316),
            Rotation2d.fromDegrees(0)
        )),
        PIECE4(new Pose2d(
            Units.inchesToMeters(blueX),
            Units.inchesToMeters(35.719),
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

    public enum Red implements PiecePosition {
        PIECE1(new Pose2d(
            Units.inchesToMeters(redX),
            Units.inchesToMeters(179.893),
            Rotation2d.fromDegrees(180)
        )),
        PIECE2(new Pose2d(
            Units.inchesToMeters(redX),
            Units.inchesToMeters(131.829),
            Rotation2d.fromDegrees(180)
        )),
        PIECE3(new Pose2d(
            Units.inchesToMeters(redX),
            Units.inchesToMeters(83.316),
            Rotation2d.fromDegrees(180)
        )),
        PIECE4(new Pose2d(
            Units.inchesToMeters(redX),
            Units.inchesToMeters(35.719),
            Rotation2d.fromDegrees(180)
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
}
