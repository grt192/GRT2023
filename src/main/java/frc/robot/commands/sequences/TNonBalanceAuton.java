package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MoverSubsystem.MoverPosition;

public class TNonBalanceAuton extends AutonAutonSequence{

    /**
     * Top most auton sequence
     * @param robotContainer robotContainer
     * @param placePose place position for 1st game piece
     * @param placePose2 place position for 2nd game piece
     * @param pickPose pick position for gamepiece on ground
     * @param height hieght of 1st game peice
     * @param hieght2 hieght of 2nd gamepiece
     */
    public TNonBalanceAuton(RobotContainer robotContainer, Pose2d initialPose, Pose2d placePose1, Pose2d pickPose, Pose2d placePose2,  MoverPosition height, MoverPosition height2){
        super(robotContainer, initialPose, placePose1, pickPose, placePose2, height, height2);
    }

}
