package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GripperConstants.*;

public class GripperSubsytem extends SubsystemBase{

    public Value state;

    DoubleSolenoid pfft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FORCHAN, REVCHAN);

    public GripperSubsytem(){
        //first set to forward
        state = Value.kForward;
    }

    //instant command method
    public void gripToggle(){
        if (state == Value.kForward){
            state = Value.kReverse;
        }
        else {
            state = Value.kForward;
        }
    }

    @Override
    public void periodic() {
        pfft.set(state);
        System.out.println(state);
    }
     }
