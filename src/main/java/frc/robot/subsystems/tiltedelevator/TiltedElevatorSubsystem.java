package frc.robot.subsystems.tiltedelevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.ShuffleboardUtil;
import frc.robot.subsystems.tiltedelevator.ElevatorState.OffsetState;

import static frc.robot.Constants.TiltedElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

public class TiltedElevatorSubsystem extends SubsystemBase {
    // Config
    private volatile boolean IS_MANUAL = false;
    private static final double OFFSET_FACTOR = 0.01; // The factor to multiply driver input by when changing the offset.
    
    // Whether to read and update shuffleboard values
    private static final boolean OVERRIDE_SHUFFLEBOARD_ENABLE = false;
    private volatile boolean SHUFFLEBOARD_ENABLE = OVERRIDE_SHUFFLEBOARD_ENABLE || Constants.GLOBAL_SHUFFLEBOARD_ENABLE;

    // States
    private ElevatorState state = ElevatorState.GROUND;
    public boolean pieceGrabbed = false;

    public OffsetState offsetState = OffsetState.DEFAULT;
    private double offsetDistMeters = 0;
    private double manualPower = 0;

    // Devices
    private final TiltedElevatorIO elevatorIO;
    private TiltedElevatorIOInputsAutoLogged inputs = new TiltedElevatorIOInputsAutoLogged();

    // Shuffleboard
    private final ShuffleboardTab shuffleboardTab;

    public TiltedElevatorSubsystem(TiltedElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;

        shuffleboardTab = Shuffleboard.getTab("Tilted Elevator");

        GenericEntry isManualEntry = shuffleboardTab.add("Is manual", IS_MANUAL)
            .withPosition(6, 0)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
        GenericEntry shuffleboardEnableEntry = shuffleboardTab.add("Shuffleboard enable", SHUFFLEBOARD_ENABLE)
            .withPosition(7, 0)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

        ShuffleboardUtil.addBooleanListener(isManualEntry, (value) -> IS_MANUAL = value);
        ShuffleboardUtil.addBooleanListener(shuffleboardEnableEntry, (value) -> SHUFFLEBOARD_ENABLE = value);
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(inputs);
        Logger.getInstance().processInputs("TiltedElevator", inputs);

        // When limit switch is pressed, reset encoder
        if (inputs.zeroLimitSwitchPressed) elevatorIO.zeroExtensionEncoder(); 

        // When magnet is detected, reset encoder
        /*
        if (leftHallSensor != null) {
            HallEffectSensor.Magnet leftHallSensorPos = leftHallSensor.getHallEffectState(inputs.currentExtensionMeters);
            // System.out.println(leftHallSensorPos);
            // System.out.println(lastHallPos + " " + leftHallSensorPos);
            if (leftHallSensorPos != null && lastHallPos == null){
                // extensionEncoder.setPosition(Units.inchesToMeters(EXTENSION_LIMIT));//leftHallSensorPos.getExtendDistanceMeters());
            }
            lastHallPos = leftHallSensorPos;
        }
        */

        // If we're in manual power mode, use percent out power supplied by driver joystick.
        if (IS_MANUAL) {
            elevatorIO.setPower(manualPower);
            return;
        }

        if (state == ElevatorState.HOME) {
            if (!inputs.zeroLimitSwitchPressed) {
                elevatorIO.setPower(-0.25);
                elevatorIO.enableReverseLimit(false);
            } else {
                elevatorIO.setPower(0);
                elevatorIO.enableReverseLimit(true);
                state = ElevatorState.GROUND;
            }
            return;
        }

        elevatorIO.enableReverseLimit(true);

        /*
        if (SHUFFLEBOARD_ENABLE) {
            ShuffleboardUtil.pollShuffleboardDouble(extensionPEntry, extensionPidController::setP);
            ShuffleboardUtil.pollShuffleboardDouble(extensionIEntry, extensionPidController::setI);
            ShuffleboardUtil.pollShuffleboardDouble(extensionDEntry, extensionPidController::setD);
            ShuffleboardUtil.pollShuffleboardDouble(extensionToleranceEntry, (value) -> extensionPidController.setSmartMotionAllowedClosedLoopError(value, 0));
            ShuffleboardUtil.pollShuffleboardDouble(rampEntry, (value) -> extensionMotor.setClosedLoopRampRate(value));
            arbFeedforward = arbFFEntry.getDouble(arbFeedforward);
        }
        */

        // If we're trying to get to 0, set the motor to 0 power so the carriage drops with gravity
        // and hits the hard stop / limit switch.
        double targetExtension = getTargetExtensionMeters();
        if (targetExtension == 0 && inputs.currentExtensionMeters < Units.inchesToMeters(1) && !inputs.zeroLimitSwitchPressed) {
            elevatorIO.setPower(-0.075);
        }
        // If we're trying to get max extension and we're currently within 1" of our goal, move elevator up so it hits the magnet
        // else if (targetExtension >= EXTENSION_LIMIT && (EXTENSION_LIMIT - currentPos < Units.inchesToMeters(1))) {
        //     extensionMotor.set(0.075);
        // } else {
            // Set PID reference
            elevatorIO.setPosition(targetExtension);
        // }

        Logger.getInstance().recordOutput("TiltedElevator/State", state.name());
        Logger.getInstance().recordOutput("TiltedElevator/OffsetState", offsetState.name());
        Logger.getInstance().recordOutput("TiltedElevator/OffsetDistMeters", offsetDistMeters);
        Logger.getInstance().recordOutput("TiltedElevator/ManualPower", manualPower);

        Logger.getInstance().recordOutput("TiltedElevator/TargetExtensionMeters", targetExtension);
        Logger.getInstance().recordOutput("TiltedElevator/AtTarget", atTarget());
    }

    /**
     * Sets the state of the subsystem.
     * @param state The `ElevatorState` to set the subsystem to.
     */
    public void setState(ElevatorState state) {
        resetOffset();
        this.state = state;
        this.offsetState = OffsetState.DEFAULT;
    }

    /**
     * Toggles the state of the subsystem between the two specified `ElevatorState`s, defaulting
     * to state 1. Also resets the distance offset.
     * 
     * @param state1 The first state.
     * @param state2 The second state.
     */
    public void toggleState(ElevatorState state1, ElevatorState state2) {
        setState(state == state1
            ? state2
            : state1);
    }

    /**
     * Gets the state of the subsystem.
     * @return The `ElevatorState` of the subsystem.
     */
    public ElevatorState getState() {
        return state;
    }

    /**
     * Changes the distance offset from given [-1.0, 1.0] driver powers.
     * @param power The power to change the offset by.
     */
    public void changeOffsetDistMeters(double power) {
        this.offsetDistMeters += OFFSET_FACTOR * power;
    }

    /**
     * Zeros the distance offset.
     */
    public void resetOffset() {
        this.offsetDistMeters = 0;
    }

    /**
     * Sets the manual power of this subsystem.
     * @param power The manual [-1.0, 1.0] driver-controlled power.
     */
    public void setManualPower(double power) {
        this.manualPower = power;
    }

    /**
     * Gets the current extension, in meters, of the subsystem.
     * @return The current extension, in meters.
     */
    public double getExtensionMeters() { 
        return inputs.currentExtensionMeters;
    }

    /**
     * Gets the target extension, in meters, of the subsystem.
     * @return The target extension, in meters.
     */
    public double getTargetExtensionMeters() {
        return state.getExtension(offsetState, this.pieceGrabbed) + offsetDistMeters;
    }

    /**
     * Gets whether the elevator has reached its target extension.
     * @return Whether the elevator has reached its target extension.
     */
    public boolean atTarget() {
        return Math.abs(getExtensionMeters() - getTargetExtensionMeters()) <= EXTENSION_TOLERANCE_METERS;
    }
}
