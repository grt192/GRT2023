package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.GripperSubsytem;

/**
 * Places 1 game piece with the pfft gripper mech.
 */
public class GripperPlaceCommand extends InstantCommand {
    public GripperPlaceCommand(GripperSubsytem gripperSubsystem) {
        super(() -> {
            System.out.print("Gripper is to open");
            gripperSubsystem.setState(Value.kForward);
            System.out.println("Gripper has opened");
        }, gripperSubsystem);
    }
}
