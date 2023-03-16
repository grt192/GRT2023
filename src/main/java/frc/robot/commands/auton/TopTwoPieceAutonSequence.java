package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class TopTwoPieceAutonSequence extends BaseAutonSequence {
    private static final PlacePosition INITIAL_POSE = PlacePosition.C2_HIGH;

    private static final FieldPosition MID_POSE_1 = FieldPosition.TOP_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.TOP_MIDPOS_2;
    private static final FieldPosition MID_POSE_3 = FieldPosition.TOP_MIDPOS_3;
    private static final FieldPosition MID_POSE_4 = FieldPosition.TOP_MIDPOS_4;

    private static final FieldPosition GRAB_POSE = FieldPosition.PIECE1;
    private static final PlacePosition PLACE_POSE2 = PlacePosition.C2_MID;

    /**
     * Non-balancing top auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     */
    public TopTwoPieceAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE, isRed);

        Pose2d initialPose = INITIAL_POSE.alignPosition.getPose(isRed);
        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);
        Pose2d midPose3 = MID_POSE_3.getPose(isRed);
        Pose2d midPose4 = MID_POSE_4.getPose(isRed);

        Pose2d grabPose = GRAB_POSE.getPose(isRed);

        Pose2d placePose2 = PLACE_POSE2.placePosition.getPose(isRed);
        ElevatorState elevatorState2 = PLACE_POSE2.elevatorState;

        addCommands(
            // Go and grab 2nd piece
            goAndGrab(initialPose, List.of(midPose1, midPose2, midPose3, midPose4), grabPose), 
            // Go and place grabbed piece
            goAndPlace(grabPose, List.of(midPose4, midPose3, midPose2), midPose1, placePose2, elevatorState2)
        );
    }
}
