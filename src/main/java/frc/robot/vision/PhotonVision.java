package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

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
public class PhotonVision {
    
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Vision Streaming");
        
    private final PhotonCamera camera;

    private final RobotPoseEstimator visionPoseEstimator;


    private static String getMJPEGName(int port) {
        return "Port_" + port + "_MJPEG_Server";
    }

    /**
     * Constructs a PhotonVision connection to the coprocessor.
     */
    public PhotonVision() {

        camera = new PhotonCamera(LIFECAM_NAME);

        // Create list of cameras and camera positions
        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(camera, cameraPos));

        try {
            visionPoseEstimator = new RobotPoseEstimator(
                new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile),
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
                camList
            );
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
    public Pair<Pose2d, Double> getRobotPose(Pose2d prevEstimatedRobotPose) {
        visionPoseEstimator.setReferencePose(prevEstimatedRobotPose); //put in odometry pose

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = visionPoseEstimator.update();

        if (result.isPresent() && result.get().getFirst() != null) {
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }
}
