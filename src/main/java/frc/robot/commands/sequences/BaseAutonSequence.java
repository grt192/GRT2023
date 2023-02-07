package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.BalancerCommand;
import frc.robot.commands.grabber.AidenIntakeCommand;
import frc.robot.commands.grabber.AidenPlaceCommand;
import frc.robot.commands.mover.JulianLevelCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * Auton Command Non-Balancing Sequence. Robot places pre-loaded piece, exits community to grab another, then places that peice
 */
public abstract class BaseAutonSequence extends SequentialCommandGroup {
    protected final BaseSwerveSubsystem swerveSubsystem;
    protected final RollerSubsystem rollerSubsystem;
    protected final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    /**
     * TODO
     * @param swerveSubsystem
     * @param rollerSubsystem
     * @param tiltedElevatorSubsystem
     */
    public BaseAutonSequence(BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);
    }

    /**
     * Goes to a position and intakes a game piece. Not used rn tho lol
     * @param intialPose The initial pose of the robot.
     * @param finalPose The destination pose of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndGrab(Pose2d initialPose, PiecePosition finalPose) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), finalPose.getPose())
            .alongWith(new JulianLevelCommand(tiltedElevatorSubsystem, ElevatorState.GROUND))
            .alongWith(new AidenIntakeCommand(rollerSubsystem));
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param intialPose The initial pose of the robot.
     * @param finalState The destination pose and elevator state of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndPlace(Pose2d initialPose, PlacePosition finalState) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), finalState.getPose())
            .alongWith(new JulianLevelCommand(tiltedElevatorSubsystem, finalState.getElevatorState())) // or .alongWith()?
            .andThen(new AidenPlaceCommand(rollerSubsystem));
    }

    /**
     * Goes to a positon to grab gamepiece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 (usually the pose that turns the robot 90)
     * @param midPose3 (usually the pose that turns the robot 90)
     * @param finalPose Destination position of robot
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndGrabTop(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midPose3, PiecePosition finalPose) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(new FollowPathCommand(swerveSubsystem, midPose2, List.of(), midPose3))
            .andThen(goAndGrab(midPose3, finalPose));
    }

    /**
     * Goes to a positon to place gamepiece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param midPose3 (usually the pose that turns the robot 90)
     * @param finalState Final state of the robot when placing the game piece.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndPlaceTop(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midPose3, PlacePosition finalState) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(new FollowPathCommand(swerveSubsystem, midPose2, List.of(), midPose3))
            .andThen(goAndPlace(midPose3, finalState));
    }

    /**
     * Goes to a positon to place a game piece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param finalPose Destination position of robot
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndGrabBottom(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, PiecePosition finalPose) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(goAndGrab(midPose2, finalPose));
    }

    /**
     * Goes to a positon to place a game piece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param finalState Final state of the robot when placing the game piece.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndPlaceBottom(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, PlacePosition finalState) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(goAndPlace(midPose2, finalState));
    }
}
