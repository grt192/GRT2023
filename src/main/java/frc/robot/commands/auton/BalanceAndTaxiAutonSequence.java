package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.balancing.DefaultBalancerCommand;
import frc.robot.commands.balancing.GoOverCommand;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class BalanceAndTaxiAutonSequence extends BaseAutonSequence {
    /**
     * Balancing auton sequence with mobility bonus.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param initialPosition The initial place position of the sequence.
     * @param isRed Whether this is a red auton path.
     */
    public BalanceAndTaxiAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        PlacePosition initialPosition, boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, initialPosition, isRed);

        addCommands(
            // Go over the charging station to taxi
            new GoOverCommand(swerveSubsystem, isRed),
            // Go and balance on charging station from the other side
            new DefaultBalancerCommand(swerveSubsystem, false)
        );
    }

    /**
     * Creates a balancing and mobility auton sequence that terminates after 14.5 seconds.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param initialPosition The initial place position of the sequence.
     * @param isRed Whether this is a red auton path.
     * @return The created auton path.
     */
    public static ParallelDeadlineGroup withDeadline(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        PlacePosition initialPosition, boolean isRed // TODO: better way of passing this
    ) {
        return new WaitCommand(14.5).deadlineWith(
            new BalanceAndTaxiAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, initialPosition, isRed)
        );
    }
}
