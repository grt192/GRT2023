package frc.robot.commands.dropping;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;

public class DropperChooserCommand{
    public DropperChooserCommand(){
    }

    public static Command choose(TiltedElevatorSubsystem tiltedElevatorSubsystem, RollerSubsystem rollerSubsystem){
        ElevatorState state = tiltedElevatorSubsystem.getState();
        if(state == ElevatorState.CONE_HIGH){
            return(new DropSequence(rollerSubsystem, tiltedElevatorSubsystem, 
            ElevatorState.CONE_HIGH_DROP, .2, .2));
        }
        if(state == ElevatorState.CONE_MID){
            return(new DropSequence(rollerSubsystem, tiltedElevatorSubsystem,
             ElevatorState.CONE_MID_DROP, .2, .2));
        }

        return(new DropSequence(rollerSubsystem, tiltedElevatorSubsystem, state, 0, 0));
    }
}
