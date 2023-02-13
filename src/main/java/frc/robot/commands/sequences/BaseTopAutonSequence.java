package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.PiecePosition;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public abstract class BaseTopAutonSequence extends BaseAutonSequence {
    /**
     * Non-balancing top auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param initialPose The initial pose of the robot.
     * @param midPose1 The first midpose of the sequence. Avoids Charging Station.
     * @param midPose2 The second midpose of the sequence. Keeps robot in same orentiation.
     * @param midpose3 The third midpose of the sequence. Turns robot 90 degrees to grab game piece from the side (top).
     * @param placeState The state of the robot when placing the first game piece (pose and elevator state).
     * @param grabPose The pose to grab the second game piece at.
     * @param placeState2 The state of the robot when placing the second game piece (pose and elevator state).
     */
    public BaseTopAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midpose3, 
        PlacePosition placeState, PiecePosition grabPose, PlacePosition placeState2
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, initialPose);

        addCommands(
            // Place preloaded game piece
            goAndPlace(initialPose, placeState),
            // Go and grab 2nd piece
            goAndGrabTopPath(placeState.getPose(), midPose1, midPose2, midpose3, grabPose), 
            // Go and place grabbed piece
            goAndPlaceTopPath(grabPose.getPose(), midPose1, midPose2, midpose3, placeState2)
        );
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
    private Command goAndGrabTopPath(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midPose3, PiecePosition finalPose) {
        return new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.GROUND)
            .alongWith(FollowPathCommand.from(swerveSubsystem, initialPose, List.of(), midPose1))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose2, List.of(), midPose3))
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
    private Command goAndPlaceTopPath(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midPose3, PlacePosition finalState) {
        return new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.GROUND)
            .alongWith(FollowPathCommand.from(swerveSubsystem, initialPose, List.of(), midPose1))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose2, List.of(), midPose3))
            .andThen(goAndPlace(midPose3, finalState));
    }
}
