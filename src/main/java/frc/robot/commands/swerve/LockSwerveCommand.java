package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class LockSwerveCommand extends InstantCommand {
    public LockSwerveCommand(BaseSwerveSubsystem swerveSubsystem) {
        super(() -> {
            swerveSubsystem.setSwerveModuleStates(
                new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)),
                new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)),
                new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)),
                new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0))
            );
        }, swerveSubsystem);
    }
}
