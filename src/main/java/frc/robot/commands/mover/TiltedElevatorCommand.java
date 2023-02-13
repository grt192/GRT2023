package frc.robot.commands.mover;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;

public class TiltedElevatorCommand extends InstantCommand {
    public TiltedElevatorCommand(TiltedElevatorSubsystem tiltedElevatorSubsystem, ElevatorState level) {
        super(() -> {
            System.out.println("Mover to " + level);
            tiltedElevatorSubsystem.setState(level);
            System.out.println("Mover at " + level);
        }, tiltedElevatorSubsystem);
    }
}
