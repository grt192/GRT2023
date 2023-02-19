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

    // ~110 for nothing, 130 for cone, 160 for cube
    private static final int CONE_PROXIMITY_THRESHOLD = 115;
    private static final int CUBE_PROXIMITY_THRESHOLD = 145;

    private static final int coneRedMax = 80;
    private static final int coneRedMin = 65;
    private static final int coneGreenMax = 140;
    private static final int coneGreenMin = 128;
    private static final int coneBlueMax = 51;
    private static final int coneBlueMin = 45;

    private static final int cubeRedMax = 65;
    private static final int cubeRedMin = 55;
    private static final int cubeGreenMax = 120;
    private static final int cubeGreenMin = 108;
    private static final int cubeBlueMax = 85;
    private static final int cubeBlueMin = 75;

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
        if (!allowOpen) return;
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

        // If we're opening and not allowed to anymore, or if we've finished opening, stop opening and start closing.
        if ((openTimer.get() > 0 && stopOpening) || openTimer.hasElapsed(OPEN_TIME_SECONDS)) {
            openTimer.stop();
            openTimer.reset();
            closeTimer.start();
        }

        // If we're closing and finished, stop closing.
        if (closeTimer.hasElapsed(CLOSE_TIME_SECONDS)) {
            closeTimer.stop();
            closeTimer.reset();
        }

        // Otherwise, open if we're opening and close if we're closing.
        openMotor.set(
            openTimer.get() > 0 ? 0.5 :
            closeTimer.get() > 0 ? -0.2 : 0
        );

        // if wheels must intake, and the limit switch is not pressed, turn on motors
        if (limitSwitch.get()) {
            leftBeak.set(rollPower);
            heldPiece = HeldPiece.EMPTY;
        } else {
            heldPiece = HeldPiece.CONE;
            leftBeak.set(Math.min(rollPower, 0.0));
        }

        HeldPiece limitPiece = getLimitSwitchPiece();
        HeldPiece proximityPiece = getProximitySensorPiece();

        // If either the limit switch or the proximity sensor have detected a piece, set
        // the piece to the detected piece, prioritizing the proximity sensor over the limit
        // switch.
        if (proximityPiece != HeldPiece.EMPTY) {
            heldPiece = proximityPiece;
        } else {
            heldPiece = limitPiece;
        }
    }

    /**
     * Gets the piece detected by the limit switch.
     * @return The piece detected by the limit switch. This is either `EMPTY` if unpressed or `CONE` if pressed.
     */
    private HeldPiece getLimitSwitchPiece() {
        return !limitSwitch.get() ? HeldPiece.CONE : HeldPiece.EMPTY;
    }

    /**
     * Gets the piece detected by the proximity measurement on the color sensor.
     * @return The piece detected by the proximity sensor.
     */
    private HeldPiece getProximitySensorPiece() {
        double dist = crolorSensor.getProximity();
        if (dist >= CUBE_PROXIMITY_THRESHOLD) return HeldPiece.CUBE;
        if (dist >= CONE_PROXIMITY_THRESHOLD) return HeldPiece.CONE;

        return HeldPiece.EMPTY;
    }

    /**
     * Gets the piece detected by the color sensor.
     * @return The piece detected by the color sensor.
     */
    private HeldPiece getColorSensorPiece() {
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
