package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.motorcontrol.MotorUtil;

import static frc.robot.Constants.RollerConstants.*;

public class RollerSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftBeak;
    private final WPI_TalonSRX rightBeak;
    private final WPI_TalonSRX openMotor;

    public final DigitalInput limitSwitch;

    private final Timer openTimer = new Timer();

    private final I2C.Port sensorPort = I2C.Port.kMXP;
    private final ColorSensorV3 crolorSensor = new ColorSensorV3(sensorPort);

    private final int coneRedMax = 80;
    private final int coneRedMin = 65;
    private final int coneGreenMax = 140;
    private final int coneGreenMin = 128;
    private final int coneBlueMax = 51;
    private final int coneBlueMin = 45;

    private final int cubeRedMax = 65;
    private final int cubeRedMin = 55;
    private final int cubeGreenMax = 120;
    private final int cubeGreenMin = 108;
    private final int cubeBlueMax = 85;
    private final int cubeBlueMin = 75;

    //for tuning
    // private final ShuffleboardTab tab = Shuffleboard.getTab("Color");
    // private final GenericEntry entry; 
    // private final GenericEntry gentry;
    // private final GenericEntry bentry;

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

        //for tuning
        // entry = tab.add("R", 0).getEntry();
        // gentry = tab.add("G", 0).getEntry();
        // bentry = tab.add("B", 0).getEntry();
    }

    /**
     * Opens the roller by starting the open timer.
     */
    public void openMotor() {
        openTimer.start();
    }

    /**
     * Gets whether there is a piece in the subsystem (whether the limit switch is pressed).
     * @return Whether the limit switch is pressed.
     */
    public boolean hasPiece() {
        return limitSwitch.get();
    }

    /**
     * Set the roller power of this subsystem.
     * @param power The power to set.
     */
    public void setRollPower(double power) {
        this.rollPower = power;
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

        Color detectedColor = crolorSensor.getColor();
        double red = detectedColor.red * 255;
        double green = detectedColor.green * 255;
        double blue = detectedColor.blue * 255;

        //for tuning
        // entry.setValue(red);
        // gentry.setValue(green);
        // bentry.setValue(blue);

        if (red > cubeRedMin && red < cubeRedMax && blue > cubeBlueMin && blue < cubeBlueMax && green < cubeGreenMax && green > cubeGreenMin){
            // System.out.println("cube");
        }
        else if (red > coneRedMin && red < coneRedMax && blue > coneBlueMin && blue < coneBlueMax && green < coneGreenMax && green > coneGreenMin){
            // System.out.println("cone");
        }
        else{
            // System.out.println("no object");
        }
    }
}
