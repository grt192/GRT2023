package frc.robot.commands.auton.test;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.commands.swerve.GoToPointCommand;
import frc.robot.commands.swerve.SwerveIdleCommand;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class TenFeetStraightLinePath extends SequentialCommandGroup {
    public TenFeetStraightLinePath(BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
    PlacePosition initialPosition, boolean isRed) {
        addRequirements(swerveSubsystem);
        addCommands(
            new InstantCommand(swerveSubsystem::resetPose),
            new InstantCommand(() -> swerveSubsystem.setVisionEnabled(false)),
            new GoToPointCommand(swerveSubsystem, new Pose2d(Units.feetToMeters(10), 0, new Rotation2d()), false),
            new SwerveIdleCommand(swerveSubsystem)
        );
    }
}
