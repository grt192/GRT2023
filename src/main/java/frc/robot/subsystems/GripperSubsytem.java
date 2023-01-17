package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GripperConstants.*;

public class GripperSubsytem extends SubsystemBase {
    private final DoubleSolenoid pfft;
    private Value state;

    public GripperSubsytem() {
        pfft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PFFT_FORWARD_ID, PFFT_REVERSE_ID);

        // first set to forward
        state = Value.kForward;
    }

    /**
     * Toggle the state of the gripper.
     */
    public void gripToggle() {
        state = state == Value.kForward
            ? Value.kReverse
            : Value.kForward;
    }

    @Override
    public void periodic() {
        pfft.set(state);
    }
}
