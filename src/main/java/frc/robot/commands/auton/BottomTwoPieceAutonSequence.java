package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class BottomTwoPieceAutonSequence extends BaseAutonSequence {
    private static final PlacePosition INITIAL_POSE = PlacePosition.A2_HIGH;

    private static final FieldPosition MID_POSE_1 = FieldPosition.BOTTOM_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.BOTTOM_MIDPOS_2;

    private static final FieldPosition GRAB_POSE = FieldPosition.PIECE4;
    private static final PlacePosition PLACE_POSE2 = PlacePosition.A2_MID;

    /**
     * Non-balancing bottom auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether this is a red auton path.
     */
    public BottomTwoPieceAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE, isRed);

        Pose2d initialPose = INITIAL_POSE.alignPosition.getPose(isRed);
        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);

        Pose2d grabPose = GRAB_POSE.getPose(isRed);

        Pose2d placePose2 = PLACE_POSE2.placePosition.getPose(isRed);
        ElevatorState elevatorState2 = PLACE_POSE2.elevatorState;

        addCommands(
            // Spline through midpose 1 to avoid overshoot
            FollowPathCommand.from(swerveSubsystem, initialPose, List.of(midPose1.getTranslation()), midPose2, false, true),
            // Go and grab 2nd piece
            goAndGrab(midPose2, List.of(), grabPose, true, false),
            // Go and place grabbed piece
            goAndPlace(grabPose, List.of(midPose2), midPose1, placePose2, elevatorState2)
        );
    }
}
