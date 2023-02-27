package frc.robot.commands.mover;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState.OffsetState;

public class TiltedElevatorCommand extends CommandBase {
    private final ElevatorState targetState;
    private final OffsetState offsetState;

    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    private final Timer timer;

    public TiltedElevatorCommand(TiltedElevatorSubsystem tiltedElevatorSubsystem, ElevatorState targetState, OffsetState offsetState) {
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        this.targetState = targetState;
        this.offsetState = offsetState;

        timer = new Timer();

        addRequirements(tiltedElevatorSubsystem);
    }

    public TiltedElevatorCommand(TiltedElevatorSubsystem tiltedElevatorSubsystem, ElevatorState targetState) {
        this(tiltedElevatorSubsystem, targetState, OffsetState.DEFAULT);
    }

    public void initialize(){
        System.out.println("Mover to " + targetState);

        tiltedElevatorSubsystem.setState(targetState);
        tiltedElevatorSubsystem.offsetState = this.offsetState;

        timer.start();
    }

    public void end(boolean interrupted) {
        System.out.println("Mover at " + targetState);
    }

    public boolean isFinished() {
        return tiltedElevatorSubsystem.atTarget() || timer.hasElapsed(3);
    }
}
