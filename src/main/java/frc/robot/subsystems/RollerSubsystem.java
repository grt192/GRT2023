package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;

import static frc.robot.Constants.RollerConstants.*;

public class RollerSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftBeak;
    private final WPI_TalonSRX rightBeak;
    private final WPI_TalonSRX openMotor;

    private final DigitalInput limitSwitch;

    private final Timer openTimer = new Timer();
    private final double OPEN_TIME_SECONDS = 1.0;
    private final double COOLDOWN_SECONDS = 2.0;

    private double rollPower = 0.0;

    public RollerSubsystem() {
        leftBeak = MotorUtil.createTalonSRX(LEFT_ID);
        leftBeak.setInverted(true);
        leftBeak.setNeutralMode(NeutralMode.Brake);

        rightBeak = MotorUtil.createTalonSRX(RIGHT_ID);
        rightBeak.follow(leftBeak);
        rightBeak.setNeutralMode(NeutralMode.Brake);

        openMotor = MotorUtil.createTalonSRX(OPEN_ID);
        openMotor.setNeutralMode(NeutralMode.Brake);
        openMotor.setInverted(true);

        limitSwitch = new DigitalInput(LIMIT_SWITCH_ID);
    }

    /**
     * Opens the roller by starting the open timer.
     */
    public void openMotor() {
        openTimer.start();
    }

    @Override
    public void periodic() {
        // If `OPEN_TIME` hasn't elapsed yet, run the motor.
        boolean opening = openTimer.get() > 0 && !openTimer.hasElapsed(OPEN_TIME_SECONDS);
        openMotor.set(opening ? 0.3 : 0);

        // If the cooldown has passed, stop and reset the timer.
        if (openTimer.hasElapsed(OPEN_TIME_SECONDS + COOLDOWN_SECONDS)) {
            openTimer.stop();
            openTimer.reset();
        }

        // if wheels must intake, and the limit switch is not pressed, turn on motors
        if (limitSwitch.get()) {
            leftBeak.set(rollPower);
        } else {
            leftBeak.set(Math.min(rollPower, 0.0));
        }
    }

    /**
     * Set the roller power of this subsystem.
     * @param power The power to set.
     */
    public void setRollPower(double power) {
        this.rollPower = power;
    }
}
