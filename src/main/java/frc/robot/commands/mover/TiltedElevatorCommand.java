package frc.robot.commands.mover;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;

public class TiltedElevatorCommand extends CommandBase {
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;
    private final ElevatorState targetState;

    public TiltedElevatorCommand(TiltedElevatorSubsystem tiltedElevatorSubsystem, ElevatorState targetState) {
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        this.targetState = targetState;

        addRequirements(tiltedElevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Mover to " + targetState);
        tiltedElevatorSubsystem.setState(targetState);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Mover at " + targetState);
    }

    @Override
    public boolean isFinished() {
        return tiltedElevatorSubsystem.atTarget();
    }
}
