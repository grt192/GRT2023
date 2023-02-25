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

public class TopOnePieceAutonSequence extends BaseAutonSequence {
    private static final FieldPosition INITIAL_POSE = FieldPosition.C2_INIT;
    private static final PlacePosition PLACE_POSE1 = PlacePosition.C2HIGH;

    private static final FieldPosition MID_POSE_1 = FieldPosition.TOP_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.TOP_MIDPOS_2;

    /**
     * Non-balancing top auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     */
    public TopOnePieceAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE.getPose(isRed)); // TODO: better

        Pose2d initialPose = INITIAL_POSE.getPose(isRed);
        PlaceState placeState1 = PLACE_POSE1.getPlaceState(isRed);

        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);

        addCommands(
            // Place preloaded game piece
            goAndPlace(initialPose, placeState1),
            // Pathfollow outside community (to grab pose) but don't turn
            FollowPathCommand.from(
                swerveSubsystem,
                initialPose,
                List.of(
                    midPose1.getTranslation()
                ),
                midPose2
            )
        );
    }
}
