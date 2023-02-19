package frc.robot.commands.mover;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;


public class TiltedElevatorCommand extends CommandBase {
    TiltedElevatorSubsystem tiltedElevatorSubsystem;

    public TiltedElevatorCommand(TiltedElevatorSubsystem tiltedElevatorSubsystem, ElevatorState level) {
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        System.out.println("Mover to " + level);
        tiltedElevatorSubsystem.setState(level);
        addRequirements(tiltedElevatorSubsystem);
    }
    
    public boolean isFinished() {
        return(tiltedElevatorSubsystem.atTarget());
    }
}
