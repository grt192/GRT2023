package frc.robot.commands.mover;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;

public class TiltedElevatorCommand extends CommandBase {
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;
    private final ElevatorState level;

    public TiltedElevatorCommand(TiltedElevatorSubsystem tiltedElevatorSubsystem, ElevatorState level) {
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        this.level = level;

        addRequirements(tiltedElevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Mover to " + level);
        tiltedElevatorSubsystem.setState(level);
    }

    @Override
    public boolean isFinished() {
        return tiltedElevatorSubsystem.atTarget();
    }
}
