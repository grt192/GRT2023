package frc.robot.commands.sequences.test;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class BoxAutonSequence extends SequentialCommandGroup {
    public BoxAutonSequence(BaseSwerveSubsystem swerveSubsystem) {
        addRequirements(swerveSubsystem);
        addCommands(
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(),
                List.of(), 
                new Pose2d(2, 0, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(2, 0, new Rotation2d()), 
                List.of(), 
                new Pose2d(2, -1, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(2, -1, new Rotation2d()), 
                List.of(), 
                new Pose2d(3, -1, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(3, -1, new Rotation2d()), 
                List.of(), 
                new Pose2d(3, 1, new Rotation2d())
            ),
            FollowPathCommand.fromReversed(
                swerveSubsystem, 
                new Pose2d(3, 1, new Rotation2d()), 
                List.of(), 
                new Pose2d(2, 1, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(2, 1, new Rotation2d()), 
                List.of(), 
                new Pose2d(2, 0, new Rotation2d())
            ),
            FollowPathCommand.fromReversed(
                swerveSubsystem, 
                new Pose2d(2, 0, new Rotation2d()), 
                List.of(), 
                new Pose2d()
            )
        );
    }
}
