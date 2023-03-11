package frc.robot.commands.auton.test;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class ContinuousBoxAutonSequence extends SequentialCommandGroup {
    public ContinuousBoxAutonSequence(BaseSwerveSubsystem swerveSubsystem) {
        addRequirements(swerveSubsystem);
        addCommands(
            new InstantCommand(swerveSubsystem::resetPose),
            new InstantCommand(() -> swerveSubsystem.setVisionEnabled(false)),
            FollowPathCommand.composedFrom(
                swerveSubsystem, 
                new Pose2d(),
                List.of(
                    new Pose2d(2, 0, new Rotation2d()),
                    new Pose2d(2, -1, new Rotation2d()),
                    new Pose2d(3, -1, new Rotation2d()),
                    new Pose2d(3, 1, new Rotation2d()),
                    new Pose2d(2, 1, new Rotation2d()),
                    new Pose2d(2, 0, new Rotation2d())
                ),
                new Pose2d()
            )
        );
    }
}
