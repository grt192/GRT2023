package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FieldUtil {
    private static double FIELD_WIDTH_METERS = Units.inchesToMeters(650.75);

    /**
     * Mirrors a pose across the game field. Use this to mirror BLUE autonomous poses to get 
     * their RED equivalents.
     * 
     * @param pose The pose to mirror.
     * @return The mirrored pose.
     */
    public static Pose2d mirrorPoseAcrossField(Pose2d pose) {
        return new Pose2d(
            FIELD_WIDTH_METERS - pose.getX(),
            pose.getY(),
            Rotation2d.fromDegrees(180).minus(pose.getRotation())
        );
    }

    /**
     * Returns whether a pose lies within the field.
     * @param pose The pose to check.
     * @return Whether the pose lies in the field.
     */
    public static boolean poseInField(Pose2d pose) {
        return pose.getX() < 16.54175 && pose.getX() > 0 && pose.getY() < 8.0137 && pose.getY() > 0; //from field json file
    }
}
