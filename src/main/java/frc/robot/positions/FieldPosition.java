package frc.robot.positions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.util.FieldUtil;
import static frc.robot.Constants.FieldConstants.*;

public enum FieldPosition {
    C3(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES),
        Units.inchesToMeters(C3_Y_INCHES), // 195.55
        Rotation2d.fromDegrees(180)
    )),
    C2(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES),
        Units.inchesToMeters(C2_Y_INCHES), // 174.123
        Rotation2d.fromDegrees(180)
    )),
    C1(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES),
        Units.inchesToMeters(C1_Y_INCHES), // 152.123
        Rotation2d.fromDegrees(180)
    )),

    B3(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES),
        Units.inchesToMeters(B3_Y_INCHES), // 129.75
        Rotation2d.fromDegrees(180)
    )),
    B2(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES),
        Units.inchesToMeters(B2_Y_INCHES), // 107.801
        Rotation2d.fromDegrees(180)
    )),
    B1(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES),
        Units.inchesToMeters(B1_Y_INCHES), // 86.149
        Rotation2d.fromDegrees(180)
    )),

    A3(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES),
        Units.inchesToMeters(A3_Y_INCHES), // 64.818
        Rotation2d.fromDegrees(180)
    )),
    A2(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES),
        Units.inchesToMeters(A2_Y_INCHES), // 41.761
        Rotation2d.fromDegrees(180)
    )),
    A1(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES),
        Units.inchesToMeters(A1_Y_INCHES), // 20.016
        Rotation2d.fromDegrees(180)
    )),

    C3_INIT(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES + ALIGNMENT_OFFSET_INCHES),
        Units.inchesToMeters(C3_Y_INCHES), // 195.55
        Rotation2d.fromDegrees(180)
    )),
    C2_INIT(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES + ALIGNMENT_OFFSET_INCHES),
        Units.inchesToMeters(C2_Y_INCHES), // 174.123
        Rotation2d.fromDegrees(180)
    )),
    C1_INIT(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES + ALIGNMENT_OFFSET_INCHES),
        Units.inchesToMeters(C1_Y_INCHES), // 152.123
        Rotation2d.fromDegrees(180)
    )),

    B3_INIT(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES + ALIGNMENT_OFFSET_INCHES),
        Units.inchesToMeters(B3_Y_INCHES), // 129.75
        Rotation2d.fromDegrees(180)
    )),
    B2_INIT(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES + ALIGNMENT_OFFSET_INCHES),
        Units.inchesToMeters(B2_Y_INCHES), // 107.801
        Rotation2d.fromDegrees(180)
    )),
    B1_INIT(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES + ALIGNMENT_OFFSET_INCHES),
        Units.inchesToMeters(B1_Y_INCHES), // 86.149
        Rotation2d.fromDegrees(180)
    )),

    A3_INIT(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES + ALIGNMENT_OFFSET_INCHES),
        Units.inchesToMeters(A3_Y_INCHES), // 64.818
        Rotation2d.fromDegrees(180)
    )),
    A2_INIT(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES + ALIGNMENT_OFFSET_INCHES),
        Units.inchesToMeters(A2_Y_INCHES), // 41.761
        Rotation2d.fromDegrees(180)
    )),
    A1_INIT(new Pose2d(
        Units.inchesToMeters(GRID_X_INCHES + ALIGNMENT_OFFSET_INCHES),
        Units.inchesToMeters(A1_Y_INCHES), // 20.016
        Rotation2d.fromDegrees(180)
    )),

    BOTTOM_MIDPOS_1(new Pose2d(
        Units.inchesToMeters(94.373),
        Units.inchesToMeters(29),
        Rotation2d.fromDegrees(180)
    )),
    BOTTOM_MIDPOS_2(new Pose2d(
        Units.inchesToMeters(170),
        Units.inchesToMeters(29), 
        Rotation2d.fromDegrees(0) // orginally 180 deg
    )),

    BOTTOM_1PIECEPOS(new Pose2d(
        Units.inchesToMeters(238.152),
        Units.inchesToMeters(29), 
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

    SPECIAL_BALANCE_POS(new Pose2d(
        Units.inchesToMeters(233.092),
        Units.inchesToMeters(105.965),
        Rotation2d.fromDegrees(180)
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
