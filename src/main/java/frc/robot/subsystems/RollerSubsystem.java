package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil;

import static frc.robot.Constants.RollerConstants.*;

import java.sql.Time;

import org.opencv.osgi.OpenCVInterface;
import org.opencv.osgi.OpenCVNativeLoader;

public class RollerSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftBeak;
    private final WPI_TalonSRX rightBeak;
    private final WPI_TalonSRX openMotor;

    private final DigitalInput limitSwitch;
    private final ColorSensorV3 crolorSensor;

    public enum HeldPiece {
        CONE, CUBE, EMPTY;
    }

    private HeldPiece heldPiece = HeldPiece.EMPTY;
    public boolean allowOpen = true;

    private final Timer openTimer = new Timer();
    private final Timer closeTimer = new Timer();
    private final Timer rollTimer = new Timer();

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
    private final double CLOSE_TIME_SECONDS = 0.5;
    private final double COOLDOWN_SECONDS = 2.0;

    private double rollPower = 0.0;
    private double rollDuration = 0.5;
    private boolean rolling = false;
    

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
        crolorSensor = new ColorSensorV3(I2C.Port.kMXP);

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
        closeTimer.stop();
        closeTimer.reset();
    }

    public InstantCommand getOuttakeCommand(double power, double duration){
        return(new InstantCommand(() -> {
            roll(power, duration);
        }));
    }

    public void roll(double power, double duration){
        rollTimer.start();
        rolling = true;
        rollPower = power;
        rollDuration = duration;
    }

    /**
     * Gets the currently held piece in the subsystem, or `HeldPiece.EMPTY` if there is no piece.
     * @return The held piece, or `HeldPiece.EMPTY`.
     */
    public HeldPiece getPiece() {
        return heldPiece;
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
        boolean opening = openTimer.get() > 0 && !openTimer.hasElapsed(OPEN_TIME_SECONDS);
        boolean closing = !opening || !allowOpen;

        if (closing) {
            closeTimer.start();
        }

        if (closeTimer.get() > 0) {
            if (!closeTimer.hasElapsed(CLOSE_TIME_SECONDS)) {
                openMotor.set(-.2);
            } else {
                openMotor.set(0);
            }
        } else {
            openMotor.set(0.5);
        }

        // If `OPEN_TIME` hasn't elapsed yet, run the motor.
        // boolean opening = openTimer.get() > 0 && !openTimer.hasElapsed(OPEN_TIME_SECONDS);
        // if (opening && allowOpen) {
        //     openMotor.set(opening && allowOpen ? 0.5 : 0);
        // }

        // If the cooldown has passed, stop and reset the timer.
        if (openTimer.hasElapsed(OPEN_TIME_SECONDS + COOLDOWN_SECONDS)) {
            openTimer.stop();
            openTimer.reset();
        }

        if(rolling){
            if(rollTimer.hasElapsed(rollDuration)){
                rollPower = 0;
                rolling = false;
                rollTimer.stop();
                rollTimer.reset();
            }
        }

        // if wheels must intake, and the limit switch is not pressed, turn on motors
        if (limitSwitch.get()) {
            leftBeak.set(rollPower);
            heldPiece = HeldPiece.EMPTY;
        } else {
            heldPiece = HeldPiece.CONE;
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

        // TODO: modify `heldPiece` based on color sensor data
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
