package frc.robot.commands.dropping;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class DropperChooserCommand extends InstantCommand {
    public DropperChooserCommand(RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, BaseSwerveSubsystem baseSwerveSubsystem) {
        super(() -> {
            getSequence(rollerSubsystem, tiltedElevatorSubsystem, baseSwerveSubsystem).schedule();
        }, rollerSubsystem, tiltedElevatorSubsystem, baseSwerveSubsystem);
    }

    /**
     * Gets the correct drop sequence to schedule for this command.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilter elevator subsystem.
     * @return The `SequentialCommandGroup` representing dropping the piece as a sequence.
     */
    private static SequentialCommandGroup getSequence(
        RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, BaseSwerveSubsystem baseSwerveSubsystem
    ) {
        ElevatorState state = tiltedElevatorSubsystem.getState();

        return switch (state) {
            case CONE_HIGH -> new DropSequence(rollerSubsystem, tiltedElevatorSubsystem, baseSwerveSubsystem,
                ElevatorState.CONE_HIGH_DROP, .2, 0, 0, .2, 0, 0);
            
            case CONE_MID -> new DropSequence(rollerSubsystem, tiltedElevatorSubsystem, baseSwerveSubsystem,
                ElevatorState.CONE_MID_DROP, .2, 0, 0, .2, 0, 0);

            case CUBE_HIGH -> new DropSequence(rollerSubsystem, tiltedElevatorSubsystem, baseSwerveSubsystem,
                ElevatorState.CUBE_HIGH, 0, .2, .5, .2, Units.inchesToMeters(5), .5);
            
            case CUBE_MID -> new DropSequence(rollerSubsystem, tiltedElevatorSubsystem, baseSwerveSubsystem,
                ElevatorState.CUBE_MID, 0, .2, .5, .2, Units.inchesToMeters(5), .5);
            
            default -> new DropSequence(rollerSubsystem, tiltedElevatorSubsystem, baseSwerveSubsystem,
                state, 0, 0, 0, 0, 0, 0);
        };
    }
}
