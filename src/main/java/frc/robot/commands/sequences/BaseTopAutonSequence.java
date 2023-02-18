package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public abstract class BaseTopAutonSequence extends BaseAutonSequence {
    private static final PlacePosition INITIAL_POSE = PlacePosition.C3MID;
    private static final FieldPosition MID_POSE_1 = FieldPosition.TOP_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.TOP_MIDPOS_2;
    private static final FieldPosition MID_POSE_3 = FieldPosition.TOP_MIDPOS_3;
    private static final FieldPosition MID_POSE_4 = FieldPosition.TOP_MIDPOS_4;

    private static final PlacePosition PLACE_POSE = PlacePosition.C2MID;
    private static final PlacePosition PLACE_POSE_2 = PlacePosition.C3MID;

    private static final FieldPosition GRAB_POSE = FieldPosition.PIECE1;

    /**
     * Non-balancing top auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     */
    public BaseTopAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE.getPlaceState(isRed).getPose()); // TODO: better

        PlaceState initialPose = INITIAL_POSE.getPlaceState(isRed);
        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);
        Pose2d midPose3 = MID_POSE_3.getPose(isRed);
        Pose2d midPose4 = MID_POSE_4.getPose(isRed);

        PlaceState placeState = PLACE_POSE.getPlaceState(isRed);
        PlaceState placeState2 = PLACE_POSE_2.getPlaceState(isRed);

        Pose2d grabPose = GRAB_POSE.getPose(isRed);

        addCommands(
            // Place preloaded game piece
            goAndPlace(initialPose.getPose(), placeState),
            // Go and grab 2nd piece
            goAndGrabTopPath(placeState.getPose(), midPose1, midPose2, midPose3, midPose4, grabPose), 
            // Go and place grabbed piece
            goAndPlaceTopPath(grabPose, midPose4, midPose3, midPose2, midPose1, placeState2)
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
    private Command goAndGrabTopPath(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midPose3, Pose2d midPose4, Pose2d finalPose) {
        return new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.GROUND)
            .alongWith(FollowPathCommand.from(swerveSubsystem, initialPose, List.of(), midPose1))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose2, List.of(), midPose3))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose3, List.of(), midPose4))
            .andThen(goAndGrab(midPose4, finalPose));
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
    private Command goAndPlaceTopPath(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midPose3, Pose2d midPose4, PlaceState finalState) {
        return new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.GROUND)
            .alongWith(FollowPathCommand.from(swerveSubsystem, initialPose, List.of(), midPose1))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose2, List.of(), midPose3))
            .andThen(FollowPathCommand.from(swerveSubsystem, midPose3, List.of(), midPose4))
            .andThen(goAndPlace(midPose4, finalState));
    }
}
