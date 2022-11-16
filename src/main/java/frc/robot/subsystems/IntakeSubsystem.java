package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {

    private WPI_TalonSRX intakeTalon;
    private double intakePower;

    private Solenoid deploySol;
    private boolean deploy;

    public IntakeSubsystem() {
        intakeTalon = new WPI_TalonSRX(intakeTalonID);
        intakeTalon.configFactoryDefault();

        deploySol = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
        deploy = false;

        this.intakePower = 0;
    }

    @Override
    public void periodic() {
        intakeTalon.set(intakePower);
        deploySol.set(deploy);
    }

    /**
     * Set power (percent output) of intake rollers.
     * @param intakePower value from -1 to 1 (pe)
     */
    public void setPower(double intakePower) {
        this.intakePower = intakePower;
    }

    public void toggleIntakeDeploy() {
        deploy = !deploy;

    }
    
}
