package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsytem extends SubsystemBase{

    public Value state;

    DoubleSolenoid pfft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    public GripperSubsytem(){
        state = Value.kForward;
    }

    @Override
    public void periodic() {
        pfft.set(state);
        System.out.println(state);
    }
}
