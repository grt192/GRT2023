package frc.robot.positions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.util.FieldUtil;

public enum FieldPosition {
    BOTTOM_INIT(new Pose2d(
        Units.inchesToMeters(75.178),
        Units.inchesToMeters(17.714),
        Rotation2d.fromDegrees(180)
    )),
    BOTTOM_MIDPOS_1(new Pose2d(
        Units.inchesToMeters(94.373),
        Units.inchesToMeters(28.007),
        Rotation2d.fromDegrees(180)
    )),
    BOTTOM_MIDPOS_2(new Pose2d(
        Units.inchesToMeters(196.000),
        Units.inchesToMeters(32.000), 
        Rotation2d.fromDegrees(180)
    )),

    BALANCE_INIT(new Pose2d(
        Units.inchesToMeters(75.178),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(180)
    )),
    BALANCE_MIDPOS_1(new Pose2d(
        Units.inchesToMeters(215.000),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(180)
    )),
    BALANCE_MIDPOS_2(new Pose2d(
        Units.inchesToMeters(250.150),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(90)
    )),
    BALANCE_MIDPOS_3(new Pose2d(
        Units.inchesToMeters(215.000),
        Units.inchesToMeters(107.638),
        Rotation2d.fromDegrees(0)
    )),

    TOP_INIT(new Pose2d(
        Units.inchesToMeters(75.178),
        Units.inchesToMeters(190.366),
        Rotation2d.fromDegrees(180)
    )),
    TOP_MIDPOS_1(new Pose2d(
        Units.inchesToMeters(94.373),
        Units.inchesToMeters(187.638),
        Rotation2d.fromDegrees(180)
    )),
    TOP_MIDPOS_2(new Pose2d(
        Units.inchesToMeters(179.577),
        Units.inchesToMeters(187.638),
        Rotation2d.fromDegrees(180)
    )),
    TOP_MIDPOS_3(new Pose2d(
        Units.inchesToMeters(221.451),
        Units.inchesToMeters(205.895),
        Rotation2d.fromDegrees(-90)
    )),
    TOP_MIDPOS_4(new Pose2d(
        Units.inchesToMeters(278.978),
        Units.inchesToMeters(205.895),
        Rotation2d.fromDegrees(-90)
    )),

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

    public final Pose2d BLUE;
    public final Pose2d RED;

    /**
     * Constructs a `FieldPosition` from a blue-team pose. The red-team pose is automatically generated
     * by mirroring the blue.
     * @param pose The blue-team pose.
     */
    private FieldPosition(Pose2d pose) {
        this.BLUE = pose;
        this.RED = FieldUtil.mirrorPoseAcrossField(pose);
    }

    /**
     * Gets the team-specific `Pose2d` represented by this field position.
     * @param isRed Whether to return the RED position. If false, returns BLUE.
     * @return The team-specific pose.
     */
    public Pose2d getPose(boolean isRed) {
        return isRed ? RED : BLUE;
    }
}
