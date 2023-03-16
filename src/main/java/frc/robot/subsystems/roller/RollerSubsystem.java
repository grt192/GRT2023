package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
    private final RollerIO rollerIO;
    private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

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

    private static final double OPEN_TIME_SECONDS = 1.0;
    private static final double CLOSE_TIME_SECONDS = 0.5;
    private static final double COOLDOWN_SECONDS = 2.0;

    private double rollPower = 0.0;

    public RollerSubsystem(RollerIO rollerIO) {
        this.rollerIO = rollerIO;

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
        rollerIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Roller", inputs);

        boolean stopOpening = !allowOpen && prevAllowedOpen; // only heed falling edge of allowed open so we can open from ground position
        prevAllowedOpen = allowOpen;

        boolean opening = openTimer.get() > 0 && !openTimer.hasElapsed(OPEN_TIME_SECONDS);
        boolean closing = !opening || stopOpening;

        if (closing) {
            closeTimer.start();
        }

        if (closeTimer.get() > 0) {
            if (!closeTimer.hasElapsed(CLOSE_TIME_SECONDS)) {
                rollerIO.setOpenPower(-0.2);
            } else {
                rollerIO.setOpenPower(0);
            }
        } else {
            rollerIO.setOpenPower(0.5);
        }

        // If the cooldown has passed, stop and reset the timer.
        if (openTimer.hasElapsed(OPEN_TIME_SECONDS + COOLDOWN_SECONDS)) {
            openTimer.stop();
            openTimer.reset();
        }

        // if wheels must intake, and the limit switch is not pressed, turn on motors
        if (!inputs.limitSwitchPressed) {
            rollerIO.setRollPower(rollPower);
        } else {
            rollerIO.setRollPower(Math.min(rollPower, 0.0));
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

        Logger.getInstance().recordOutput("Roller/RollPower", rollPower);
        Logger.getInstance().recordOutput("Roller/HeldPiece", heldPiece.name());
    }

    /**
     * Gets the piece detected by the limit switch.
     * @return The piece detected by the limit switch. This is either `EMPTY` if unpressed or `CONE` if pressed.
     */
    private HeldPiece getLimitSwitchPiece() {
        return inputs.limitSwitchPressed ? HeldPiece.CONE : HeldPiece.EMPTY;
    }

    /**
     * Gets the piece detected by the proximity measurement on the color sensor.
     * @return The piece detected by the proximity sensor.
     */
    private HeldPiece getProximitySensorPiece() {
        if (inputs.proximitySensorReading >= CUBE_PROXIMITY_THRESHOLD) return HeldPiece.CUBE;
        if (inputs.proximitySensorReading >= CONE_PROXIMITY_THRESHOLD) return HeldPiece.CONE;
        return HeldPiece.EMPTY;
    }

    /**
     * Gets the piece detected by the color sensor.
     * @return The piece detected by the color sensor.
     */
    private HeldPiece getColorSensorPiece() {
        double red = inputs.colorSensorRed;
        double green = inputs.colorSensorGreen;
        double blue = inputs.colorSensorBlue;

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
