package frc.robot.commands.sequences.test;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class RotatingSCurveAutonSequence extends SequentialCommandGroup {
    public RotatingSCurveAutonSequence(BaseSwerveSubsystem swerveSubsystem) {
        addRequirements(swerveSubsystem);
        addCommands(
            FollowPathCommand.from(
                swerveSubsystem, 
                new Pose2d(), 
                List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
                ),
                new Pose2d(3, 0, Rotation2d.fromDegrees(90))
            ),
            FollowPathCommand.from(
                swerveSubsystem,
                new Pose2d(3, 0, Rotation2d.fromDegrees(90)),
                List.of(
                    new Translation2d(2, -1),
                    new Translation2d(1, 1)
                ),
                new Pose2d()
            )
        );
    }
}
