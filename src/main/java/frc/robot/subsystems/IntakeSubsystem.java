package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

    private WPI_TalonSRX intakeTalon;
    private double intakePower;

    public IntakeSubsystem() {
        intakeTalon = new WPI_TalonSRX(intakeTalonID);
        intakeTalon.configFactoryDefault();

        this.intakePower = 0;
    }

    @Override
    public void periodic() {
        intakeTalon.set(intakePower);
    }

    /**
     * Set power (percent output) of intake rollers.
     * @param intakePower value from -1 to 1 (pe)
     */
    public void setPower(double intakePower) {
        this.intakePower = intakePower;
    }
    
}
