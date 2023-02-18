package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.commands.BalancerCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class BaseBalanceAutonSequence extends BaseAutonSequence {
    private static final PlacePosition INITIAL_POSE = PlacePosition.B2MID;
    private static final FieldPosition MID_POSE_1 = FieldPosition.BALANCE_MIDPOS_1;
    private static final FieldPosition MID_POSE_2 = FieldPosition.BALANCE_MIDPOS_2;
    private static final FieldPosition MID_POSE_3 = FieldPosition.BALANCE_MIDPOS_3;

    /**
     * Balancing auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether this is a red auton path.
     */
    public BaseBalanceAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE.getPlaceState(isRed).getPose()); // TODO: better

        PlaceState initialPose = INITIAL_POSE.getPlaceState(isRed);
        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);
        Pose2d midPose3 = MID_POSE_3.getPose(isRed);


        addCommands(
            // Place preloaded game piece
            Place(initialPose.getElevatorState()),
            // Go out of community
            FollowPathCommand.from(swerveSubsystem, initialPose.getPose(), List.of(), midPose1),
            //do 180
            FollowPathCommand.from(swerveSubsystem, midPose1, List.of(), midPose2),
            FollowPathCommand.from(swerveSubsystem, midPose2, List.of(), midPose3),
            // Go and balance on charging station
            new BalancerCommand(swerveSubsystem)
        );
    }
}
