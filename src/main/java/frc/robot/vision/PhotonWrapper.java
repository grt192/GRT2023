package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.VisionConstants.*;

/**
 * A connection to PhotonVision on the coprocessor.
 */
public class PhotonWrapper {
    
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Vision Streaming");
        

    private ArrayList<PhotonPoseEstimator> photonPoseEstimators;

    /**
     * Constructs a PhotonVision connection to the coprocessor.
     */
    public PhotonWrapper() {

        try {
            for (Pair<PhotonCamera, Transform3d> camera : CAMERA_LIST)
            {
                photonPoseEstimators.add(new PhotonPoseEstimator(
                    new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile),
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
                    camera.getFirst(),
                    camera.getSecond()
                ));
            }
            
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Gets the estimated robot pose from vision. Call this every periodic loop to update the drivetrain pose 
     * estimator in `SwerveSubsystem` with vision data.
     * 
     * @param prevEstimatedRobotPose The last odometry robot pose estimate, for setting the vision reference pose.
     * @return A tuple representing [estimated pose, timestamp].
     */
    public ArrayList<EstimatedRobotPose> getRobotPose(Pose2d prevEstimatedRobotPose) {        
        ArrayList<EstimatedRobotPose> estimatedPoses = new ArrayList<EstimatedRobotPose>();

        for (PhotonPoseEstimator estimator: photonPoseEstimators) {
            estimator.setReferencePose(prevEstimatedRobotPose); // input odometry pose
            Optional<EstimatedRobotPose> estimatedPose = estimator.update();

            if (estimatedPose.isPresent()) {
                estimatedPoses.add(estimatedPose.get());
            } 
        }

        return estimatedPoses;
    }
}