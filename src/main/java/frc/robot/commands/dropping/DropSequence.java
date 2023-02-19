package frc.robot.commands.dropping;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;

public class DropSequence extends SequentialCommandGroup{
    public DropSequence(
        RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, 
        ElevatorState dropHeight, double waitTime1, double waitTime2
    ) {
        addRequirements(rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            new TiltedElevatorCommand(tiltedElevatorSubsystem, dropHeight),
            new WaitCommand(waitTime1),
            new InstantCommand(rollerSubsystem::openMotor, rollerSubsystem),
            new WaitCommand(waitTime2),
            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.GROUND)
        );
    }
}
