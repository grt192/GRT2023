package frc.robot.commands.auton.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.grabber.VisionAlignmentCommand;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class AutoAlignmentAuton extends SequentialCommandGroup {
    public AutoAlignmentAuton(BaseSwerveSubsystem swerveSubsystem) {
        addRequirements(swerveSubsystem);
        addCommands(
            new VisionAlignmentCommand(swerveSubsystem)
        );
    }
}
