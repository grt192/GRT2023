package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static frc.robot.Constants.VisionConstants.*;

/**
 * A connection to PhotonVision on the coprocessor.
 */
public class PhotonWrapper {
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator backPoseEstimator;

    private final boolean SHUFFLEBOARD_ON = true;

    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("PhotonVision");

    private final GenericEntry frontStatusEntry = shuffleboardTab.add("front tag?", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 0).getEntry();
    private final GenericEntry xPosFrontEntry = shuffleboardTab.add("xpos front", 0).withPosition(0, 1).getEntry();
    private final GenericEntry yPosFrontEntry = shuffleboardTab.add("ypos front", 0).withPosition(1, 1).getEntry();
    private final GenericEntry timestampFrontEntry = shuffleboardTab.add("timestamp front", 0).withPosition(2, 1).getEntry();

    private final GenericEntry backStatusEntry = shuffleboardTab.add("back tag?", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(0, 2).getEntry();
    private final GenericEntry xPosBackEntry = shuffleboardTab.add("xpos back", 0).withPosition(0, 3).getEntry();
    private final GenericEntry yPosBackEntry = shuffleboardTab.add("ypos back", 0).withPosition(1, 3).getEntry();
    private final GenericEntry timestampBackEntry = shuffleboardTab.add("timestamp back", 0).withPosition(2, 3).getEntry();


    /**
     * Constructs a PhotonVision connection to the coprocessor.
     */
    public PhotonWrapper() {

        try {
            frontPoseEstimator = new PhotonPoseEstimator(
                // new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile),
                // new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2023-chargedup.json"),
                AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(),
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                FRONT_CAMERA,
                FRONT_CAMERA_POSE
            );

            backPoseEstimator = new PhotonPoseEstimator(
                AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(),
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                BACK_CAMERA,
                BACK_CAMERA_POSE
            );
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Get estimated robot pose from a single photon camera. 
     * @param prevEstimatedRobotPose The last odometry robot pose estimate
     * @param poseEstimator The estimated vision pose.
     * @return
     */
    public EstimatedRobotPose getRobotPose(Pose2d prevEstimatedRobotPose, PhotonPoseEstimator poseEstimator) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose); // input odometry pose
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update();

        if (estimatedPose.isPresent()) {
            return estimatedPose.get();
        }
        return null;
    }

    /**
     * Gets the estimated robot poses from vision. Call this every periodic loop to update the drivetrain pose 
     * estimator in `SwerveSubsystem` with vision data.
     * 
     * @param prevEstimatedRobotPose The last odometry robot pose estimate, for setting the vision reference pose.
     * @return A list of estimated vision poses.
     */
    public List<EstimatedRobotPose> getRobotPoses(Pose2d prevEstimatedRobotPose) {
        EstimatedRobotPose frontPose = getRobotPose(prevEstimatedRobotPose, frontPoseEstimator);
        EstimatedRobotPose backPose = getRobotPose(prevEstimatedRobotPose, backPoseEstimator);

        // Update Shuffleboard
        if (SHUFFLEBOARD_ON) {
            frontStatusEntry.setBoolean(frontPose != null);
            backStatusEntry.setBoolean(backPose != null);

            if (frontPose != null) {
                xPosFrontEntry.setValue(Units.metersToInches(frontPose.estimatedPose.getX()));
                yPosFrontEntry.setValue(Units.metersToInches(frontPose.estimatedPose.getY()));
                timestampFrontEntry.setValue(frontPose.timestampSeconds);
            }
            
            if (backPose != null) {
                xPosBackEntry.setValue(Units.metersToInches(backPose.estimatedPose.getX()));
                yPosBackEntry.setValue(Units.metersToInches(backPose.estimatedPose.getY()));
                timestampBackEntry.setValue(backPose.timestampSeconds);
            }
        }

        List<EstimatedRobotPose> outputPoses = new ArrayList<EstimatedRobotPose>();
        if (frontPose != null) outputPoses.add(frontPose);
        if (backPose != null) outputPoses.add(backPose);
        return outputPoses;
    }
}
