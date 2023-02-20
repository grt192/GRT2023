package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class TopAutonSequence extends BaseAutonSequence {
    private static final PlacePosition INITIAL_POSE = PlacePosition.C3MID;
    private static final FieldPosition MID_POSE_1 = FieldPosition.TOP_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.TOP_MIDPOS_2;
    private static final FieldPosition MID_POSE_3 = FieldPosition.TOP_MIDPOS_3;
    private static final FieldPosition MID_POSE_4 = FieldPosition.TOP_MIDPOS_4;

    private static final PlacePosition PLACE_POSE = PlacePosition.C2MID;

    private static final FieldPosition GRAB_POSE = FieldPosition.PIECE1;

    /**
     * Non-balancing top auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     */
    public TopAutonSequence(
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

        Pose2d grabPose = GRAB_POSE.getPose(isRed);

        addCommands(
            // Place preloaded game piece
            Place(initialPose.getElevatorState()),
            // Go and grab 2nd piece
            goAndGrab(initialPose.getPose(), List.of(midPose1, midPose2, midPose3, midPose4), grabPose), 
            // Go and place grabbed piece
            goAndPlace(grabPose, List.of(midPose4, midPose3, midPose2, midPose1), placeState)
        );
    }
}
