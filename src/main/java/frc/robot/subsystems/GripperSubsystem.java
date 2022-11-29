package frc.robot.subsystems;

import static frc.robot.Constants.GripperConstants.SOLENOIDPORT;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {
    public boolean open;
    Solenoid pfft = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOIDPORT);

    public GripperSubsystem() {
        open = false;
    }

    public void periodic() {
        pfft.set(open);
    }
}
