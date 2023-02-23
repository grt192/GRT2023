package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class BottomAutonSequence extends BaseAutonSequence {
    private static final FieldPosition INITIAL_POSE = FieldPosition.A2_INIT;
    private static final PlacePosition PLACE_POSE1 = PlacePosition.A2MID;

    private static final FieldPosition MID_POSE_1 = FieldPosition.BOTTOM_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.BOTTOM_MIDPOS_2;

    private static final FieldPosition GRAB_POSE = FieldPosition.PIECE4;
    private static final PlacePosition PLACE_POSE2 = PlacePosition.A2HIGH;

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
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE.getPose(isRed)); // TODO: better

        Pose2d initialPose = INITIAL_POSE.getPose(isRed);
        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);

        PlaceState placeState1 = PLACE_POSE1.getPlaceState(isRed);
        PlaceState placeState2 = PLACE_POSE2.getPlaceState(isRed);

        Pose2d grabPose = GRAB_POSE.getPose(isRed);

        addCommands(
            // Place preloaded gamepiece
            goAndPlace(initialPose, placeState1),
            // Spline through midpose 1 to avoid overshoot
            FollowPathCommand.from(swerveSubsystem, initialPose, List.of(midPose1.getTranslation()), midPose2, false, true),
            // Go and grab 2nd piece
            goAndGrab(midPose2, List.of(), grabPose, true, false),
            // Go directly to midpose 2 to avoid angle wrap heading error
            FollowPathCommand.from(swerveSubsystem, grabPose, List.of(), midPose2),
            // Go and place grabbed piece
            goAndPlace(midPose2, List.of(), midPose1, placeState2)
            // goAndPlace(grabPose, List.of(midPose2), midPose1, placeState2)
        );
    }
}
