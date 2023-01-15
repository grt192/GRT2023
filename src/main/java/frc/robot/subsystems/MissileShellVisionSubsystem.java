package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.vision.PhotonVision;

public class MissileShellVisionSubsystem extends SubsystemBase {
    private final PhotonVision vision;
    private Pose2d pose = new Pose2d();

    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Vision Testing");
    private final GenericEntry xEntry = shuffleboardTab.add("Pose x", 0.0).getEntry();
    private final GenericEntry yEntry = shuffleboardTab.add("Pose y", 0.0).getEntry();

    public MissileShellVisionSubsystem(PhotonVision vision) {
        this.vision = vision;
    }

    @Override
    public void periodic() {
        var res = vision.getRobotPose(pose);
        Pose2d pose = res.getFirst();
        if (pose == null) {
            System.out.println("null");
            return;
        }
        xEntry.setDouble(Units.metersToInches(pose.getX()));
        yEntry.setDouble(Units.metersToInches(pose.getY()));
        
        System.out.println("pose X" + pose.getX());

        this.pose = pose;
    }
}
