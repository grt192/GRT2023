package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.GripperSubsytem;

/**
 * [no use] Places 1 game piece with the pfft gripper mech.
 */
public class MatthewPlaceCommand extends InstantCommand {
    public MatthewPlaceCommand(GripperSubsytem gripperSubsystem) {
        super(() -> {
            System.out.print("Gripper is to open");
            gripperSubsystem.setState(Value.kForward);
            System.out.println("Gripper has opened");
        }, gripperSubsystem);
    }
}
