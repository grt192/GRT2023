package frc.robot.subsystems;

import static frc.robot.Constants.GripperConstants.*;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class GripperSubsystem extends SubsystemBase {
    public Value open;
    DoubleSolenoid pfft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SOLENOIDPORT, SOLENOIDPORT2 );

    public GripperSubsystem() {
        open = Value.kForward;
    }

    public void periodic() {
        pfft.set(open);
        // System.out.println(open);
    }
}
