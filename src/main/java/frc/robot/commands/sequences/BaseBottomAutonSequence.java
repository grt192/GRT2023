package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public abstract class BaseBottomAutonSequence extends BaseAutonSequence {
    private static final FieldPosition INITIAL_POSE = FieldPosition.BOTTOM_INIT;
    private static final FieldPosition MID_POSE_1 = FieldPosition.BOTTOM_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.BOTTOM_MIDPOS_2;

    private static final PlacePosition PLACE_POSE = PlacePosition.A2HIGH;
    private static final PlacePosition PLACE_POSE_2 = PlacePosition.A2MID;

    private static final FieldPosition GRAB_POSE = FieldPosition.PIECE4;

    /**
     * Non-balancing auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether this is a red auton path.
     */
    public BaseBottomAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE.getPose(isRed)); // TODO: better

        Pose2d initialPose = INITIAL_POSE.getPose(isRed);
        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);

        PlaceState placeState = PLACE_POSE.getPlaceState(isRed);
        PlaceState placeState2 = PLACE_POSE_2.getPlaceState(isRed);

        Pose2d grabPose = GRAB_POSE.getPose(isRed);

        addCommands(
            // Place preloaded game piece
            goAndPlace(initialPose, placeState),
            // Go and grab 2nd piece
            goAndGrabBottomPath(placeState.getPose(), midPose1, midPose2, grabPose),
            // Go and place grabbed piece
            goAndPlaceBottomPath(grabPose, midPose2, midPose1, placeState2)
        );
    }

    /**
     * Goes to a positon to place a game piece, avoiding the charging station and turning 180 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param finalPose Destination position of robot
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndGrabBottomPath(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d finalPose) {
        return new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.GROUND)
            .alongWith(FollowPathCommand.from(swerveSubsystem, initialPose, List.of(), midPose1))
            .andThen(new PrintCommand("hit MidPose1"))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(new PrintCommand("hit MidPose2"))
            .andThen(goAndGrab(midPose2, finalPose));
    }

    /**
     * Goes to a positon to place a game piece, avoiding the charging station and turning 180 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param finalState Final state of the robot when placing the game piece.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    protected Command goAndPlaceBottomPath(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, PlaceState finalState) {
        return new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.GROUND)
            .alongWith(FollowPathCommand.from(swerveSubsystem, initialPose, List.of(), midPose1))
            .andThen(new PrintCommand("hit MidPose1"))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(new PrintCommand("hit MidPose2"))
            .andThen(goAndPlace(midPose2, finalState));
    }
}
