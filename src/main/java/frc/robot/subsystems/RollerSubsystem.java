package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.MotorUtil;

import static frc.robot.Constants.RollerConstants.*;

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
    private boolean prevAllowedOpen = true;

    private final Timer openTimer = new Timer();
    private final Timer closeTimer = new Timer();

    // ~110. for nothing, 130 for cone, 160 for cube
    private final int CONE_PROXIMITY_THRESHOLD = 115;
    private final int CUBE_PROXIMITY_THRESHOLD = 145;

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

    public RollerSubsystem() {
        leftBeak = MotorUtil.createTalonSRX(LEFT_ID);
        leftBeak.setInverted(true);
        // leftBeak.setInverted(false);
        leftBeak.setNeutralMode(NeutralMode.Brake);

        rightBeak = MotorUtil.createTalonSRX(RIGHT_ID);
        rightBeak.follow(leftBeak);
        rightBeak.setInverted(InvertType.OpposeMaster);
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

    /**
     * Set the roller power of this subsystem.
     * @param power The power to set.
     */
    public void setRollPower(double power) {
        this.rollPower = power;
    }

    @Override
    public void periodic() {
        boolean stopOpening = !allowOpen && prevAllowedOpen; // only heed falling edge of allowed open so we can open from ground position
        prevAllowedOpen = allowOpen;

        boolean opening = openTimer.get() > 0 && !openTimer.hasElapsed(OPEN_TIME_SECONDS);
        boolean closing = !opening || stopOpening;

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

        // If the cooldown has passed, stop and reset the timer.
        if (openTimer.hasElapsed(OPEN_TIME_SECONDS + COOLDOWN_SECONDS)) {
            openTimer.stop();
            openTimer.reset();
        }

        // if wheels must intake, and the limit switch is not pressed, turn on motors
        if (limitSwitch.get()) {
            leftBeak.set(rollPower);
            heldPiece = HeldPiece.EMPTY;
        } else {
            heldPiece = HeldPiece.CONE;
            leftBeak.set(Math.min(rollPower, 0.0));
        }

        HeldPiece limitPiece = detectElementsLimitSwitch();
        HeldPiece proximityPiece = detectElementsProximity();

        // if either limit or proxim sensor see a piece, use that
        if (proximityPiece != HeldPiece.EMPTY) {
            heldPiece = proximityPiece;
        } else if (limitPiece != HeldPiece.EMPTY) {
            heldPiece = limitPiece;
        } else {
            heldPiece = HeldPiece.EMPTY;
        }
    }

    public HeldPiece detectElementsLimitSwitch() {
        if (limitSwitch.get()) {
            return HeldPiece.EMPTY;
        } else {
            return HeldPiece.CONE;
        }
    }

    public HeldPiece detectElementsProximity() {
        double dist = crolorSensor.getProximity();

        // System.out.println(dist);

        if (dist >= CUBE_PROXIMITY_THRESHOLD) {
            return HeldPiece.CUBE;
        } else if (dist >= CONE_PROXIMITY_THRESHOLD) {
            return HeldPiece.CONE;
        } else {
            return HeldPiece.EMPTY;
        }
    }

    public HeldPiece detectElementsColor() {
        Color detectedColor = crolorSensor.getColor();
        double red = detectedColor.red * 255;
        double green = detectedColor.green * 255;
        double blue = detectedColor.blue * 255;

        // for tuning
        // entry.setValue(red);
        // gentry.setValue(green);
        // bentry.setValue(blue);

        // TODO: modify `heldPiece` based on color sensor data
        if (red > cubeRedMin && red < cubeRedMax && blue > cubeBlueMin && blue < cubeBlueMax && green < cubeGreenMax && green > cubeGreenMin){
            // System.out.println("cube");
            return HeldPiece.CUBE;
        }
        else if (red > coneRedMin && red < coneRedMax && blue > coneBlueMin && blue < coneBlueMax && green < coneGreenMax && green > coneGreenMin){
            // System.out.println("cone");
            return HeldPiece.CONE;
        }
        else{
            // System.out.println("no object");
            return HeldPiece.EMPTY;
        }
    }

    /**
     * Gets the currently held piece in the subsystem, or `HeldPiece.EMPTY` if there is no piece.
     * @return The held piece, or `HeldPiece.EMPTY`.
     */
    public HeldPiece getPiece() {
        return heldPiece;
    }
}
