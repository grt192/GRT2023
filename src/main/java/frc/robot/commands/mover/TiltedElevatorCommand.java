package frc.robot.commands.mover;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;


public class TiltedElevatorCommand extends CommandBase {
    TiltedElevatorSubsystem tiltedElevatorSubsystem;
    ElevatorState targetState;

    public TiltedElevatorCommand(TiltedElevatorSubsystem tiltedElevatorSubsystem, ElevatorState targetState) {
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        this.targetState = targetState;
        addRequirements(tiltedElevatorSubsystem);
    }

    public void initialize(){
        System.out.println("Mover to " + targetState);
        tiltedElevatorSubsystem.setState(targetState);
    }

    public void end(boolean interrupted) {
        System.out.println("Mover at " + targetState);
    }
    
    public boolean isFinished() {
        return(tiltedElevatorSubsystem.atTarget());
    }
}
