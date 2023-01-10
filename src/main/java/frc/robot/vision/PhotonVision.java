package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.VisionConstants.*;

/**
 * A connection to PhotonVision on the coprocessor.
 */
public class PhotonVision {
    private final PhotonCamera camera;

    private final RobotPoseEstimator visionPoseEstimator;

    /**
     * Constructs a PhotonVision connection to the coprocessor.
     */
    public PhotonVision() {
        camera = new PhotonCamera(cameraName);

        // Create list of cameras and camera positions
        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(camera, cameraPos));

        // TODO: is this hacky?
        try {
            visionPoseEstimator = new RobotPoseEstimator(
                new AprilTagFieldLayout(AprilTagFields.k2022RapidReact.m_resourceFile), // TODO: update for 2023 charged up
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
        visionPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = visionPoseEstimator.update();

        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }
}
