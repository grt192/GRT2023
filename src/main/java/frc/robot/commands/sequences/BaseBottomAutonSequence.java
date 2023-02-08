package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.PiecePosition;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public abstract class BaseBottomAutonSequence extends BaseAutonSequence {
    /**
     * Non-balancing bottom auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem Thetilted elevator subsystem.
     * @param initialPose The initial pose of the robot.
     * @param midPose1 The first midpose of the sequence. Avoids the charging station.
     * @param midPose2 The second midpose of the sequence. Keeps robot in same orientation to not hit stuff.
     * @param placeState The state of the robot when placing the first game piece (pose and elevator state).
     * @param grabPose The pose to grab the second game piece at.
     * @param placeState2 The state of the robot when placing the second game piece (pose and elevator state).
     */
    public BaseBottomAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, 
        PlacePosition placeState, PiecePosition grabPose, PlacePosition placeState2
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, initialPose);

        addCommands(
            // Place preloaded game piece
            goAndPlaceBottomPath(initialPose, midPose1, midPose2, placeState),
            // Go and grab 2nd piece
            goAndGrabBottomPath(placeState.getPose(), midPose1, midPose2, grabPose),
            // Go and place grabbed piece
            goAndPlaceBottomPath(grabPose.getPose(), midPose1, midPose2, placeState2)
        );
    }

    /**
     * Goes to a positon to place a game piece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param finalPose Destination position of robot
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndGrabBottomPath(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, PiecePosition finalPose) {
        return FollowPathCommand.fromReversed(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(FollowPathCommand.fromReversed(swerveSubsystem, midPose1, List.of(), midPose2))
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
    protected Command goAndPlaceBottomPath(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, PlacePosition finalState) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(goAndPlace(midPose2, finalState));
    }
}
