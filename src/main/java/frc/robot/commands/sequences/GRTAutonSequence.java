package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class GRTAutonSequence extends SequentialCommandGroup {
    public GRTAutonSequence(BaseSwerveSubsystem swerveSubsystem) {
        addRequirements(swerveSubsystem);
        addCommands(
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(),
                List.of(),
                new Pose2d(1, 1, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(1, 1, new Rotation2d()),
                List.of(),
                new Pose2d(),
                true
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(),
                List.of(
                    new Translation2d(0.5, -1)
                ),
                new Pose2d(1, 0, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(1, 0, new Rotation2d()),
                List.of(),
                new Pose2d(0.5, 0, new Rotation2d()),
                true
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(0.5, 0, new Rotation2d()),
                List.of(),
                new Pose2d(1.5, -1, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(1.5, -1, new Rotation2d()),
                List.of(),
                new Pose2d(1.5, 1, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(1.5, 1, new Rotation2d()),
                List.of(
                    new Translation2d(2.5, 0.5)
                ),
                new Pose2d(1.5, 0, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(1.5, 0, new Rotation2d()),
                List.of(),
                new Pose2d(2.5, -1, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(2.5, -1, new Rotation2d()),
                List.of(),
                new Pose2d(4, -1, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(4, -1, new Rotation2d()),
                List.of(),
                new Pose2d(4, 1, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(4, 1, new Rotation2d()),
                List.of(),
                new Pose2d(4.5, 1, new Rotation2d())
            ),
            new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(4.5, 1, new Rotation2d()),
                List.of(),
                new Pose2d(3.5, 1, new Rotation2d()),
                true
            )
        );
    }
}
