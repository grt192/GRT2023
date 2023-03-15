package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

@FunctionalInterface
public interface AutonFactoryFunction {
    Command apply(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed
    );
}
