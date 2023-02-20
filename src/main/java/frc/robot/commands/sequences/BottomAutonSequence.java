package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class BottomAutonSequence extends BaseAutonSequence {
    private static final PlacePosition INITIAL_POSE = PlacePosition.A1MID;
    private static final FieldPosition MID_POSE_1 = FieldPosition.BOTTOM_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.BOTTOM_MIDPOS_2;

    private static final PlacePosition PLACE_POSE = PlacePosition.A2MID;

    private static final FieldPosition GRAB_POSE = FieldPosition.PIECE4;

    /**
     * Non-balancing bottom auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether this is a red auton path.
     */
    public BottomAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE.getPlaceState(isRed).getPose()); // TODO: better

        PlaceState initialPose = INITIAL_POSE.getPlaceState(isRed);
        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);

        PlaceState placeState = PLACE_POSE.getPlaceState(isRed);

        Pose2d grabPose = GRAB_POSE.getPose(isRed);

        addCommands(
            // Place preloaded game piece
            Place(initialPose.getElevatorState()),
            // Go and grab 2nd piece
            goAndGrab(initialPose.getPose(), List.of(midPose1, midPose2), grabPose),
            // Go and place grabbed piece
            goAndPlace(grabPose, List.of(midPose2, midPose1), placeState)
        );
    }
}
