package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.balancing.DefaultBalancerCommand;
import frc.robot.commands.balancing.GoOverCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class BalanceAndTaxiAutonSequence extends BaseAutonSequence {
    private static final FieldPosition INITIAL_POSE = FieldPosition.B2_INIT;
    private static final PlacePosition PLACE_POSE = PlacePosition.B2HIGH;

    /**
     * Balancing auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether this is a red auton path.
     */
    public BalanceAndTaxiAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE.getPose(isRed)); // TODO: better

        Pose2d initialPose = INITIAL_POSE.getPose(isRed);
        PlaceState placeState = PLACE_POSE.getPlaceState(isRed);

        addCommands(
            // Interrupt balancer after 14.5 seconds have elapsed in the sequence.
            // TODO: very ugly way of implementing this at the moment.
            new WaitCommand(14.5).deadlineWith(
                // Place preloaded gamepiece
                goAndPlace(initialPose, placeState).andThen(
                    // Go over the charging station to taxi
                    new GoOverCommand(swerveSubsystem),
                    // Go and balance on charging station from the other side
                    new DefaultBalancerCommand(swerveSubsystem, true)
                )
            )
        );
    }
}
