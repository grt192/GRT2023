package frc.robot.commands.dropping;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class DropSequence extends SequentialCommandGroup{
    public DropSequence(
        RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, BaseSwerveSubsystem baseSwerveSubsystem,
        ElevatorState dropHeight, double waitTime1, double outtakePower, double outtakeDuration,
         double waitTime2, double reverseDist, double waitTime3
    ) {
        addRequirements(rollerSubsystem, tiltedElevatorSubsystem);
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

        Pose2d currentPos = baseSwerveSubsystem.getRobotPosition();

        Pose2d targetPos = currentPos.plus(new Transform2d(new Translation2d(-reverseDist * currentPos.getRotation().getCos(),
                                                                             -reverseDist * currentPos.getRotation().getSin()),
                                                                             new Rotation2d()));

        

        addCommands(
            new TiltedElevatorCommand(tiltedElevatorSubsystem, dropHeight),
            new WaitCommand(waitTime1),
            new InstantCommand(rollerSubsystem::openMotor, rollerSubsystem),
            rollerSubsystem.getOuttakeCommand(-outtakePower, outtakeDuration),
            new WaitCommand(waitTime2),
            FollowPathCommand.from(baseSwerveSubsystem, currentPos, List.of(), targetPos),
            new WaitCommand(waitTime3),
            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.GROUND)
        );
    }
}
