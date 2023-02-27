package frc.robot.commands.dropping;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class DropperChooserCommand extends InstantCommand {
    public DropperChooserCommand(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        super(() -> {
            getSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem).schedule();
        }, rollerSubsystem, tiltedElevatorSubsystem, swerveSubsystem);
    }

    /**
     * Gets the drop sequence corresponding to the current ElevatorState.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilter elevator subsystem.
     * @return The `SequentialCommandGroup` representing dropping the piece as a sequence.
     */
    public static SequentialCommandGroup getSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        ElevatorState state = tiltedElevatorSubsystem.getState();
        return getSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, state);
    }

    /**
     * Gets the drop sequence corresponding to a given target ElevatorState.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilter elevator subsystem.
     * @param state The target ElevatorState.
     * @return The `SequentialCommandGroup` representing dropping the piece as a sequence.
     */
    public static SequentialCommandGroup getSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        ElevatorState state
    ) {
        return switch (state) {
            case CONE_HIGH -> new DropSequence(
                swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                0, 0, 0, 0.2, 0.5
            );

            case CONE_MID -> new DropSequence(
                swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                0, 0, 0, 0.2, 0.5
            );

            case CUBE_HIGH -> new DropSequence(
                swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                0, 0.2, 0.5, 0.2, 0.5
            );

            case CUBE_MID -> new DropSequence(
                swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                0, 0.2, 0.5, 0.2, 0.5
            );

            default -> new DropSequence(
                swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                0, 0, 0, 0, 0.5
            );
        };
    }
}
