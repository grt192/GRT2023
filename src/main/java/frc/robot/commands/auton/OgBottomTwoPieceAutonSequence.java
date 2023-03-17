package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class OgBottomTwoPieceAutonSequence extends BaseAutonSequence {
    private static final PlacePosition INITIAL_POSE = PlacePosition.A2_HIGH;

    private static final FieldPosition MID_POSE_1 = FieldPosition.BOTTOM_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.BOTTOM_MIDPOS_2;
    private static final FieldPosition MID_POSE_3 = FieldPosition.BOTTOM_MIDPOS_3;

    private static final FieldPosition GRAB_POSE = FieldPosition.PIECE4;
    private static final PlacePosition PLACE_POSE2 = PlacePosition.A2_MID;

    /**
     * Non-balancing bottom auton sequence.
     * Turns early
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether this is a red auton path.
     */
    public OgBottomTwoPieceAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE, isRed);

        Pose2d initialPose = INITIAL_POSE.alignPosition.getPose(isRed);
        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);
        Pose2d midPose3 = MID_POSE_3.getPose(isRed);

        Pose2d grabPose = GRAB_POSE.getPose(isRed);

        Pose2d placePose2 = PLACE_POSE2.placePosition.getPose(isRed);
        ElevatorState elevatorState2 = PLACE_POSE2.elevatorState;

        addCommands(
            // Spline through midPose 1 to avoid overshoot
            FollowPathCommand.from(swerveSubsystem, initialPose, List.of(midPose1.getTranslation()), midPose2, false, true),
            FollowPathCommand.from(swerveSubsystem, midPose2, List.of(), midPose3, true, true),
            // Go and grab 2nd piece
            goAndGrab(midPose3, List.of(), grabPose, true, false),
            // Go and place grabbed piece
            FollowPathCommand.from(swerveSubsystem, grabPose, List.of(), midPose2, false, true),
            FollowPathCommand.from(swerveSubsystem, midPose2, List.of(), midPose1, true, true),
            //goAndPlace(midPose2, List.of(midPose1), initialPose, placeState2)
            goAndPlace(midPose1, placePose2, elevatorState2, true)
        );
    }
}
