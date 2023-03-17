package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static frc.robot.Constants.PfftRollerConstants.*;

public class PfftRollerSubsystem extends RollerSubsystem{
    private final DoubleSolenoid pfftOpen;
    private final DoubleSolenoid pfftClose;

    public PfftRollerSubsystem(){
        pfftOpen = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PFFT_FORWARD, PFFT_FORWARD_ATMOSPHERE);
        pfftClose = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PFFT_REVERSE_ATMOSPHERE, PFFT_REVERSE_ATMOSPHERE);
    }
    private void openRoller(){

    }
    private void clampRoller(){

    }
    private void neitherRoller(){

    }
}
