package frc.robot.commands.mover;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.ElevatorState.OffsetState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class TiltedElevatorCommand extends CommandBase {
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    private ElevatorState targetState = null;
    private OffsetState offsetState = null;

    private final Timer timer;

    public TiltedElevatorCommand(TiltedElevatorSubsystem tiltedElevatorSubsystem, ElevatorState targetState) {
        this(tiltedElevatorSubsystem);

        this.targetState = targetState;
    }

    public TiltedElevatorCommand(TiltedElevatorSubsystem tiltedElevatorSubsystem, OffsetState offsetState) {
        this(tiltedElevatorSubsystem);

        this.offsetState = offsetState;
    }

    private TiltedElevatorCommand(TiltedElevatorSubsystem tiltedElevatorSubsystem) {
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;
        this.timer = new Timer();

        addRequirements(tiltedElevatorSubsystem);
    }

    public void initialize() {
        System.out.println("Mover to " + targetState);

        if (targetState != null) tiltedElevatorSubsystem.setState(targetState);
        if (offsetState != null) tiltedElevatorSubsystem.offsetState = offsetState;

        timer.start();
    }

    public void end(boolean interrupted) {
        System.out.println("Mover at " + targetState);
    }

    public boolean isFinished() {
        return tiltedElevatorSubsystem.atTarget() || timer.hasElapsed(3);
    }
}
