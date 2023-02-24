package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.balancing.DefaultBalancerCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class BalanceAutonSequence extends BaseAutonSequence {
    private static final FieldPosition INITIAL_POSE = FieldPosition.B2_INIT;
    private static final PlacePosition PLACE_POSE = PlacePosition.B2HIGH;

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
    public BalanceAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE.getPose(isRed)); // TODO: better

        Pose2d initialPose = INITIAL_POSE.getPose(isRed);
        PlaceState placeState = PLACE_POSE.getPlaceState(isRed);

        Pose2d midPose1 = MID_POSE_1.getPose(isRed);
        Pose2d midPose2 = MID_POSE_2.getPose(isRed);
        Pose2d midPose3 = MID_POSE_3.getPose(isRed);

        addCommands(
            // Interrupt balancer after 14.5 seconds have elapsed in the sequence.
            // TODO: very ugly way of implementing this at the moment.
            new WaitCommand(14.5).deadlineWith(
                // Place preloaded gamepiece
                goAndPlace(initialPose, placeState).andThen(
                    // Go and balance on charging station
                    new DefaultBalancerCommand(swerveSubsystem)
                )
            )
            // Go out of community and do 180
            // FollowPathCommand.composedFrom(
            //     swerveSubsystem,
            //     initialPose,
            //     List.of(midPose1, midPose2),
            //     midPose3
            // ),
        );
    }
}
