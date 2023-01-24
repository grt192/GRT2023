package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static frc.robot.Constants.VisionConstants.*;

/**
 * A connection to PhotonVision on the coprocessor.
 */
public class PhotonWrapper {
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Vision Streaming");

    private List<PhotonPoseEstimator> photonPoseEstimators;

    /**
     * Constructs a PhotonVision connection to the coprocessor.
     */
    public PhotonWrapper() {
        photonPoseEstimators = CAMERA_LIST.stream().map((camera) -> {
            // try {
                return new PhotonPoseEstimator(
                    // new AprilTagFieldLayout("/src/main/java/frc/robot/vision/2023-chargedup.json"),
                    // new AprilTagFieldLayout("/src/main/java/frc/robot/vision/2023-chargedup.json"),
                    // new AprilTagFieldLayout("https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2023-chargedup.json"),
                    new AprilTagFieldLayout(Arrays.asList(
                        new AprilTag(1, 
                            new Pose3d(
                                new Translation3d(15.513558, 1.071626, 0.462788), 
                                new Rotation3d(new Quaternion(0, 0, 0, 1)))),
                        new AprilTag(2, 
                            new Pose3d(
                                new Translation3d(15.513558, 2.748026, 0.462788), 
                                new Rotation3d(new Quaternion(0, 0, 0, 1))))),
                        16.54175,
                        8.0137),
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
                    camera.getFirst(),
                    camera.getSecond()
                );
            // } catch (IOException e) {
                //throw new RuntimeException(e);
            // }
        }).toList();
    }

    /**
     * Gets the estimated robot poses from vision. Call this every periodic loop to update the drivetrain pose 
     * estimator in `SwerveSubsystem` with vision data.
     * 
     * @param prevEstimatedRobotPose The last odometry robot pose estimate, for setting the vision reference pose.
     * @return A list of estimated vision poses.
     */
    public List<EstimatedRobotPose> getRobotPose(Pose2d prevEstimatedRobotPose) {        
        ArrayList<EstimatedRobotPose> estimatedPoses = new ArrayList<EstimatedRobotPose>();

        for (PhotonPoseEstimator estimator : photonPoseEstimators) {
            estimator.setReferencePose(prevEstimatedRobotPose); // input odometry pose
            Optional<EstimatedRobotPose> estimatedPose = estimator.update();

            if (estimatedPose.isPresent()) {
                estimatedPoses.add(estimatedPose.get());
            }
        }

        return estimatedPoses;
    }
}
