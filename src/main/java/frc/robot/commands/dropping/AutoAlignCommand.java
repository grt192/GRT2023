package frc.robot.commands.dropping;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class AutoAlignCommand extends InstantCommand {
    public AutoAlignCommand(BaseSwerveSubsystem swerveSubsystem, PlacePosition placePosition, boolean isRed) {
        super(() -> {
            Pose2d initialPose = swerveSubsystem.getRobotPosition();

            FollowPathCommand.from(
                swerveSubsystem,
                initialPose,
                List.of(),
                placePosition.alignPosition.getPose(isRed)
            ).schedule();
        }, swerveSubsystem);
    }
}
