package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class BottomOnePieceAutonSequence extends BaseAutonSequence {
    private static final PlacePosition INITIAL_POSE = PlacePosition.A2HIGH;

    private static final FieldPosition MID_POSE_1 = FieldPosition.BOTTOM_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.BOTTOM_MIDPOS_2;
    private static final FieldPosition FINAL_POSE = FieldPosition.BOTTOM_1PIECEPOS;

    /**
     * Non-balancing bottom auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether this is a red auton path.
     */
    public BottomOnePieceAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE, isRed);

        Pose2d initialPose = INITIAL_POSE.alignPosition.getPose(isRed);
        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);
        Pose2d finalPose = FINAL_POSE.getPose(isRed);

        addCommands(
            // Pathfollow outside community (to grab pose) but don't turn
            FollowPathCommand.from(
                swerveSubsystem,
                initialPose,
                List.of(midPose1.getTranslation(), midPose2.getTranslation()),
                finalPose
            )
        );
    }
}
