package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.GripperSubsytem;

/**
 * Intakes 1 game piece with the pfft gripper mech.
 */
public class MatthewIntakeCommand extends InstantCommand {
    public MatthewIntakeCommand(GripperSubsytem gripperSubsystem) {
        super(() -> {
            System.out.print("Gripper is to close");
            gripperSubsystem.setState(Value.kReverse);
            System.out.println("Gripper has closed");
        }, gripperSubsystem);
    }
}
