package frc.robot.commands.sequences;

import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.MoverConstants;
import frc.robot.commands.BalancerCommand;
import frc.robot.commands.MatthewIntakeCommand;
import frc.robot.commands.MatthewPlaceCommand;
import frc.robot.commands.ShiraLevelCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.GripperSubsytem;
import frc.robot.subsystems.MoverSubsystem;
import frc.robot.subsystems.MoverSubsystem.MoverPosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

/**
 * Auton Command Non-Balancing Sequence. Robot places pre-loaded piece, exits community to grab another, then places that peice
 * 
 */

public class AutonAutonSequence extends SequentialCommandGroup{
    private final RobotContainer robotContainer;
    private final BaseSwerveSubsystem swerveSubsystem;
    private final GripperSubsytem gripperSubsytem;
    private final MoverSubsystem moverSubsystem;

    //non balancing auton sequence
    public AutonAutonSequence(RobotContainer robotContainer, Pose2d initialPose, Pose2d placePose, Pose2d grabPose, Pose2d placePose2, MoverPosition moverPosition, MoverPosition moverPosition2){
        this.robotContainer = robotContainer;

        swerveSubsystem = robotContainer.getSwerveSubsystem();
        gripperSubsytem = robotContainer.getGripperSubsytem();
        moverSubsystem = robotContainer.getMoverSubsystem();
 
        addRequirements(swerveSubsystem, gripperSubsytem, moverSubsystem);

        addCommands(
            //place preloaded gamepiece
            goAndPlace(initialPose, placePose, moverPosition),
            //go and grab 2nd piece
            goAndGrab(placePose, grabPose),
            //go and place grabbed piece
            goAndPlace(grabPose, placePose2, moverPosition2)

        );
    }
    // balancing auton sequence
    public AutonAutonSequence(RobotContainer robotContainer, Pose2d initialPose, Pose2d placePose, Pose2d outsidePose, MoverPosition moverPosition){
        this.robotContainer = robotContainer;

        swerveSubsystem = robotContainer.getSwerveSubsystem();
        gripperSubsytem = robotContainer.getGripperSubsytem();
        moverSubsystem = robotContainer.getMoverSubsystem();

        addRequirements(swerveSubsystem, gripperSubsytem, moverSubsystem);

        addCommands(
            //place pre loaded gamepiece
            goAndPlace(initialPose, placePose, moverPosition),
            //go out of community
            new FollowPathCommand(swerveSubsystem, placePose, null, outsidePose),
            //go and balance on charging station
            new BalancerCommand(swerveSubsystem)
        );

    }

    //make followpath and Shiralevel parallel commands?

    public Command goAndGrab(Pose2d initialPose, Pose2d finalPose){
        addCommands(        
            //get to gamepiece
            new FollowPathCommand(swerveSubsystem, initialPose, null, finalPose),
            //get mover to ground height
            new ShiraLevelCommand(moverSubsystem, MoverPosition.GROUND),
            //grab game piece
            new MatthewIntakeCommand(gripperSubsytem)
        );
        return null;
    }

    public Command goAndPlace(Pose2d intialPose, Pose2d finalPose, MoverPosition moverPosition){
        addCommands(
        //get to place location
        new FollowPathCommand(swerveSubsystem, intialPose, null, finalPose),
        //get mover to right height
        new ShiraLevelCommand(moverSubsystem, moverPosition),
        //place gamepiece
        new MatthewPlaceCommand(gripperSubsytem)
        );
        return null;
    }
}
