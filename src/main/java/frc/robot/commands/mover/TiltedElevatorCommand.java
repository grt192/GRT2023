package frc.robot.commands.mover;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;

public class TiltedElevatorCommand extends CommandBase {
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;
    private final ElevatorState targetState;
    private final Timer timer;

    public TiltedElevatorCommand(TiltedElevatorSubsystem tiltedElevatorSubsystem, ElevatorState targetState) {
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        this.targetState = targetState;
        timer = new Timer();

        addRequirements(tiltedElevatorSubsystem);
    }

    public void initialize(){
        System.out.println("Mover to " + targetState);
        tiltedElevatorSubsystem.setState(targetState);
        timer.start();
    }

    public void end(boolean interrupted) {
        System.out.println("Mover at " + targetState);
    }
    
    public boolean isFinished() {
        return tiltedElevatorSubsystem.atTarget() || timer.hasElapsed(1);
    }
}
