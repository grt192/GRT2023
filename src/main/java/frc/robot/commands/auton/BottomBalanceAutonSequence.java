package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.balancing.DefaultBalancerCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class BottomBalanceAutonSequence extends BaseAutonSequence {
    private static final FieldPosition INITIAL_POSE = FieldPosition.A2_INIT;
    private static final PlacePosition PLACE_POSE1 = PlacePosition.A2HIGH;

    private static final FieldPosition MID_POSE_1 = FieldPosition.BOTTOM_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.BOTTOM_MIDPOS_2;
    private static final FieldPosition MID_POSE_3 = FieldPosition.BOTTOM_1PIECEPOS;
    private static final FieldPosition FINAL_POSE = FieldPosition.SPECIAL_BALANCE_POS;

    /**
     * Non-balancing bottom auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether this is a red auton path.
     */
    public BottomBalanceAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE.getPose(isRed)); // TODO: better

        Pose2d initialPose = INITIAL_POSE.getPose(isRed);
        PlaceState placeState1 = PLACE_POSE1.getPlaceState(isRed);

        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);
        Pose2d midPose3 = MID_POSE_3.getPose(isRed);
        Pose2d finalPose = FINAL_POSE.getPose(isRed);

        addCommands(
            // Place preloaded game piece
            goAndPlace(initialPose, placeState1),
            // Pathfollow outside community (to final pose) but don't turn
            FollowPathCommand.from(
                swerveSubsystem,
                initialPose,
                List.of(midPose1.getTranslation(), midPose2.getTranslation(), midPose3.getTranslation()),
                finalPose
            ),
            // Go onto charging station and balance from the other side
            new DefaultBalancerCommand(swerveSubsystem, true)
        );
    }
}
