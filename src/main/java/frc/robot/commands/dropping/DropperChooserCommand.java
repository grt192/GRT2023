package frc.robot.commands.dropping;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;

public class DropperChooserCommand extends InstantCommand {
    public DropperChooserCommand(RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem) {
        super(() -> {
            getSequence(rollerSubsystem, tiltedElevatorSubsystem).schedule();
        }, rollerSubsystem, tiltedElevatorSubsystem);
    }

    /**
     * Gets the correct drop sequence to schedule for this command.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilter elevator subsystem.
     * @return The `SequentialCommandGroup` representing dropping the piece as a sequence.
     */
    private static SequentialCommandGroup getSequence(
        RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem
    ) {
        ElevatorState state = tiltedElevatorSubsystem.getState();
        if(state == ElevatorState.CONE_HIGH){
            return(new DropSequence(rollerSubsystem, tiltedElevatorSubsystem, 
            ElevatorState.CONE_HIGH_DROP, 0, .2));
        }
        if(state == ElevatorState.CONE_MID){
            return(new DropSequence(rollerSubsystem, tiltedElevatorSubsystem,
             ElevatorState.CONE_MID_DROP, .4, .2));
        }

        return switch (state) {
            case CONE_HIGH -> new DropSequence(rollerSubsystem, tiltedElevatorSubsystem, ElevatorState.CONE_HIGH_DROP, .2, .2);
            case CONE_MID -> new DropSequence(rollerSubsystem, tiltedElevatorSubsystem, ElevatorState.CONE_MID_DROP, .2, .2);
            default -> new DropSequence(rollerSubsystem, tiltedElevatorSubsystem, state, 0, 0);
        };
    }
}
