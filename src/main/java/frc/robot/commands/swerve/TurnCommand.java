package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class TurnCommand extends CommandBase{
    private final BaseSwerveSubsystem swerveSubsystem;
    private final Rotation2d targetRotation;

    public TurnCommand(BaseSwerveSubsystem swerveSubsystem, Rotation2d targetRotation){
        this.swerveSubsystem = swerveSubsystem;
        this.targetRotation = targetRotation;
    }
    
}
