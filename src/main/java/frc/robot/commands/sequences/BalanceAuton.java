package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.RobotContainer;
import frc.robot.subsystems.MoverSubsystem.MoverPosition;

public class BalanceAuton extends BaseAutonSequence {
    public BalanceAuton(RobotContainer robotContainer, Pose2d initialPose, Pose2d placePose, Pose2d outsidePose, MoverPosition height){
        super(robotContainer, initialPose, placePose, outsidePose, height );
    }
}
