package frc.robot.commands.auton.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.swerve.GoToPointCommand;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class GoToOriginSequence extends SequentialCommandGroup {
    public GoToOriginSequence(BaseSwerveSubsystem swerveSubsystem) {
        addRequirements(swerveSubsystem);
        addCommands(
            new GoToPointCommand(
                swerveSubsystem, 
                new Pose2d()
            )
        );
    }
}
