package frc.robot.positions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.util.FieldUtil;

public enum FieldPosition {
    C3_INIT(new Pose2d(
        Units.inchesToMeters(86.855),
        Units.inchesToMeters(195.55),
        Rotation2d.fromDegrees(180)
    )),
    C2_INIT(new Pose2d(
        Units.inchesToMeters(86.855),
        Units.inchesToMeters(174.123),
        Rotation2d.fromDegrees(180)
    )),
    C1_INIT(new Pose2d(
        Units.inchesToMeters(86.855),
        Units.inchesToMeters(152.123),
        Rotation2d.fromDegrees(180)
    )),

    B3_INIT(new Pose2d(
        Units.inchesToMeters(86.855),
        Units.inchesToMeters(129.75),
        Rotation2d.fromDegrees(180)
    )),
    B2_INIT(new Pose2d(
        Units.inchesToMeters(86.855),
        Units.inchesToMeters(107.801),
        Rotation2d.fromDegrees(180)
    )),
    B1_INIT(new Pose2d(
        Units.inchesToMeters(86.855),
        Units.inchesToMeters(86.149),
        Rotation2d.fromDegrees(180)
    )),

    A3_INIT(new Pose2d(
        Units.inchesToMeters(86.855),
        Units.inchesToMeters(64.818),
        Rotation2d.fromDegrees(180)
    )),
    A2_INIT(new Pose2d(
        Units.inchesToMeters(86.855),
        Units.inchesToMeters(41.761),
        Rotation2d.fromDegrees(180)
    )),
    A1_INIT(new Pose2d(
        Units.inchesToMeters(86.855),
        Units.inchesToMeters(20.016),
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
