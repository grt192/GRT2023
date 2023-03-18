package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import static frc.robot.Constants.PfftRollerConstants.*;

import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;

public class PfftRollerSubsystem extends RollerSubsystem{
    private final DoubleSolenoid pfftOpen;
    private final DoubleSolenoid pfftClose;

    private Value forwardState;
    private Value reverseState;


    public PfftRollerSubsystem(){

        pfftOpen = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PFFT_FORWARD, PFFT_FORWARD_ATMOSPHERE);
        pfftClose = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PFFT_REVERSE, PFFT_REVERSE_ATMOSPHERE);
        forwardState = Value.kReverse;
        reverseState = Value.kReverse;
    }
    //when theres nothing
    @Override
    protected void openRoller(){
        forwardState = Value.kForward;
        reverseState = Value.kReverse;
    }
    //goes to atmospheric
    @Override
    protected void closeRoller(){
        forwardState = Value.kReverse;
        reverseState = Value.kReverse;
    }
    @Override
    protected void activeclampRoller(){
        forwardState = Value.kReverse;
        reverseState = Value.kForward;
    }
    //when theres a cube
    @Override
    protected void neitherRoller(){
        forwardState = Value.kReverse;
        reverseState = Value.kReverse;
    }
    @Override
    public void periodic() {
        super.periodic();
        pfftOpen.set(forwardState);
        pfftClose.set(reverseState);
    }
}
