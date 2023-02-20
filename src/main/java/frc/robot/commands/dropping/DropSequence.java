package frc.robot.commands.dropping;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

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
        ElevatorState dropHeight, double waitTime1, double outtakePower, double outtakeDuration, double waitTime2, double backTime
    ) {
        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            new TiltedElevatorCommand(tiltedElevatorSubsystem, dropHeight),
            new WaitCommand(waitTime1),
            new InstantCommand(rollerSubsystem::openMotor, rollerSubsystem),
            rollerSubsystem.getOuttakeCommand(-outtakePower, outtakeDuration),
            new WaitCommand(waitTime2),
            new InstantCommand(() -> swerveSubsystem.setDrivePowers(-.2, 0, 0, true)),
            new WaitCommand(backTime),
            new InstantCommand(() -> swerveSubsystem.setDrivePowers(0, 0, 0, true)),
            new InstantCommand(() -> tiltedElevatorSubsystem.setState(ElevatorState.GROUND))
        );
    }
}
