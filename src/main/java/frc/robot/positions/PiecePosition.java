package frc.robot.positions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public enum PiecePosition {
    PIECE1(new Pose2d(
        Units.inchesToMeters(278.264),
        Units.inchesToMeters(179.893),
        Rotation2d.fromDegrees(-90)
    )),
    PIECE2(new Pose2d(
        Units.inchesToMeters(278.264),
        Units.inchesToMeters(131.829),
        Rotation2d.fromDegrees(0)
    )),
    PIECE3(new Pose2d(
        Units.inchesToMeters(278.264),
        Units.inchesToMeters(83.316),
        Rotation2d.fromDegrees(0)
    )),
    PIECE4(new Pose2d(
        Units.inchesToMeters(278.264),
        Units.inchesToMeters(35.719),
        Rotation2d.fromDegrees(0)
    ));

    public final Pose2d RED;
    public final Pose2d BLUE;

    private PiecePosition(Pose2d pose) {
        // TODO: mirror
        this.RED = pose;
        this.BLUE = pose;
    }
}
