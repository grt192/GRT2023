package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.MoverSubsystem;
import frc.robot.subsystems.MoverSubsystem.MoverPosition;

public class ShiraLevelCommand extends InstantCommand {
    public ShiraLevelCommand(MoverSubsystem moverSubsystem, MoverPosition level) {
        super(() -> {
            System.out.println("Mover to " + level);
            moverSubsystem.setState(level);
            System.out.println("Mover at " + level);
        }, moverSubsystem);
    }
}
