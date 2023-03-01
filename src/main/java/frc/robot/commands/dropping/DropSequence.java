package frc.robot.commands.dropping;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.grabber.RollerPlaceCommand;
import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState.OffsetState;

public class DropSequence extends SequentialCommandGroup {
    /*
     * Order of commands
     * 1. Moves to dropHeight
     * 2. Waits waitTime1 (dont want to be moving when letting go)
     * 3. Releases gamepiece and outtakes at outtakePower for outtakeDuration
     * 4. Waits waitTime2 (dont want to be moving before piece is fully out)
     * 5. Moves backwards reverseDist
     * 6. Waits waitTime3
     * 7. Moves to GROUND
     */
    public DropSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        double waitAfterAtDropHeight, double outtakePower, double outtakeDuration, double waitAfterPlacing, double backTime
    ) {
        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            new TiltedElevatorCommand(tiltedElevatorSubsystem, OffsetState.DROPPING),
            new WaitCommand(waitAfterAtDropHeight),
            new RollerPlaceCommand(rollerSubsystem, -outtakePower, outtakeDuration),
            new WaitCommand(waitAfterPlacing),
            new InstantCommand(() -> swerveSubsystem.setDrivePowers(-0.2, 0, 0, true)),
            new WaitCommand(backTime),
            new InstantCommand(() -> swerveSubsystem.setDrivePowers(0, 0, 0, true)),
            new InstantCommand(() -> tiltedElevatorSubsystem.setState(ElevatorState.GROUND))
        );
    }
}
