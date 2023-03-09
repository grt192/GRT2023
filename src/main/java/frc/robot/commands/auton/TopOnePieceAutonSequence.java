package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class TopOnePieceAutonSequence extends BaseAutonSequence {
    private static final PlacePosition INITIAL_POSE = PlacePosition.C2HIGH;

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
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE, isRed);

        Pose2d initialPose = INITIAL_POSE.alignPosition.getPose(isRed);
        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);

        addCommands(
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
