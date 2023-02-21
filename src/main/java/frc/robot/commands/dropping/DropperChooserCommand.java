package frc.robot.commands.dropping;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class DropperChooserCommand extends InstantCommand {
    public DropperChooserCommand(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        super(() -> {
            getSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem).schedule();
        }, rollerSubsystem, tiltedElevatorSubsystem, swerveSubsystem);
    }

    /**
     * Gets the correct drop sequence to schedule for this command.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilter elevator subsystem.
     * @return The `SequentialCommandGroup` representing dropping the piece as a sequence.
     */
    private static SequentialCommandGroup getSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        ElevatorState state = tiltedElevatorSubsystem.getState();

        return switch (state) {
            case CONE_HIGH -> new DropSequence(
                swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                ElevatorState.CONE_HIGH_DROP, 0, 0, 0, .2, 0.5
            );

            case CONE_MID -> new DropSequence(
                swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                ElevatorState.CONE_MID_DROP, 0, 0, 0, .2, 0.5
            );

            case CUBE_HIGH -> new DropSequence(
                swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                ElevatorState.CUBE_HIGH, 0, .2, .5, .2, .5
            );

            case CUBE_MID -> new DropSequence(
                swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                ElevatorState.CUBE_MID, 0, .2, .5, .2, .5
            );

            default -> new DropSequence(
                swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem,
                state, 0, 0, 0, 0, 0.5
            );
        };
    }
}
