package frc.robot.commands.mover;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.PivotElevatorSubsystem;
import frc.robot.subsystems.PivotElevatorSubsystem.MoverPosition;

public class ShiraLevelCommand extends InstantCommand {
    public ShiraLevelCommand(PivotElevatorSubsystem moverSubsystem, MoverPosition level) {
        super(() -> {
            System.out.println("Mover to " + level);
            moverSubsystem.setState(level);
            System.out.println("Mover at " + level);
        }, moverSubsystem);
    }
}
