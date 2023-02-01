package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GripperConstants.*;

public class GripperSubsytem extends SubsystemBase {
    private final DoubleSolenoid pfftL;
    private final DoubleSolenoid pfftR;

    private Value state = Value.kForward;

    public GripperSubsytem() {
        pfftL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PFFT_FORWARD_IDL, PFFT_REVERSE_IDL);
        pfftR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PFFT_FORWARD_IDR, PFFT_REVERSE_IDR);
    }

    /**
     * Toggle the state of the gripper.
     */
    public void gripToggle() {
        state = state == Value.kForward
            ? Value.kReverse
            : Value.kForward;
    }

    public Value getState() {
        return state;
    }

    public void setState(Value state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        pfftL.set(state);
        pfftR.set(state);
    }
}
