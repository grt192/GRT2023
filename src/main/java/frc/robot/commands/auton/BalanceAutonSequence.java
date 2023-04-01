package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.balancing.DefaultBalancerCommand;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class BalanceAutonSequence extends BaseAutonSequence {
    /**
     * Balancing auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param initialPosition The initial place position of the sequence.
     * @param isRed Whether this is a red auton path.
     */
    public BalanceAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        PlacePosition initialPosition, boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, initialPosition, isRed);

        addCommands(
            new WaitCommand(1.4), //to let elevator lower all the way untested
            // Go and balance on charging station
            new DefaultBalancerCommand(swerveSubsystem, true)
        );
    }

    /**
     * Creates a balancing auton sequence that terminates after 14.5 seconds.
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
            new BalanceAutonSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, initialPosition, isRed)
        );
    }
}
