package frc.robot.commands.auton.test;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class TwentyFeetStraightLinePath extends SequentialCommandGroup {
    public TwentyFeetStraightLinePath(BaseSwerveSubsystem swerveSubsystem) {
        addRequirements(swerveSubsystem);
        addCommands(
            new InstantCommand(swerveSubsystem::resetPose),
            FollowPathCommand.from(
                swerveSubsystem, 
                new Pose2d(), 
                List.of(), 
                new Pose2d(Units.feetToMeters(20), 0, new Rotation2d())
            )
        );
    }
}
