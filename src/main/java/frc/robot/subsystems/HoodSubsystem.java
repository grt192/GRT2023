package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    private Solenoid hoodSol;
    private boolean hoodUp;

    public HoodSubsystem()
    {
        hoodSol = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
        this.hoodUp = true;
    }

    public void periodic() {
        hoodSol.set(hoodUp);
    }
    
}
