package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class SwerveIdleCommand extends InstantCommand {
    public SwerveIdleCommand(BaseSwerveSubsystem swerveSubsystem) {
        super(() -> {
            /*
            swerveSubsystem.setSwerveModuleStates(
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            );
            */
            swerveSubsystem.setDrivePowers(0, 0, 0, true);
        }, swerveSubsystem);
    }
}
