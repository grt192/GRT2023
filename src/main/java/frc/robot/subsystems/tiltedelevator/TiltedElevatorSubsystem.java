package frc.robot.subsystems.tiltedelevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.MotorUtil;
import frc.robot.util.ShuffleboardUtil;
import frc.robot.motorcontrol.HallEffectSensor;
import frc.robot.subsystems.tiltedelevator.ElevatorState.OffsetState;
import frc.robot.motorcontrol.HallEffectMagnet;

import static frc.robot.Constants.TiltedElevatorConstants.*;

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

    private boolean hallPressed = false;
    private HallEffectMagnet lastHallPos = null;

    // Devices
    private final CANSparkMax extensionMotor;
    private RelativeEncoder extensionEncoder;
    private SparkMaxPIDController extensionPidController;

    private final CANSparkMax extensionFollow;
    private final CANSparkMax extensionFollowB;

    private final DigitalInput zeroLimitSwitch;
    private final HallEffectSensor leftHallSensor;

    // Constants
    private static final double EXTENSION_GEAR_RATIO = 14.0 / 64.0;
    private static final double EXTENSION_CIRCUMFERENCE = Units.inchesToMeters(Math.PI * 0.500); // approx circumference of winch
    private static final double EXTENSION_ROTATIONS_TO_METERS = EXTENSION_GEAR_RATIO * EXTENSION_CIRCUMFERENCE * 2.0 * (15.0 / 13.4);

    private static final double extensionP = 2.4;
    private static final double extensionI = 0;
    private static final double extensionD = 0;
    private static final double extensionTolerance = 0.003;
    private static final double extensionRampRate = 0.4;
    private double arbFeedforward = Constants.IS_R1 ? 0.03 : 0;

    // Shuffleboard
    private final ShuffleboardTab shuffleboardTab;
    private final GenericEntry 
        extensionPEntry, extensionIEntry, extensionDEntry,
        extensionToleranceEntry, arbFFEntry, rampEntry;
    private final GenericEntry manualPowerEntry, targetExtensionEntry;
    private final GenericEntry currentExtensionEntry, currentVelEntry, currentStateEntry, offsetDistEntry;
    private final GenericEntry limitSwitchEntry, hallEntry;

    public TiltedElevatorSubsystem() {
        extensionMotor = MotorUtil.createSparkMax(EXTENSION_ID, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake); 
            sparkMax.setInverted(true);
            sparkMax.setClosedLoopRampRate(extensionRampRate);

            extensionEncoder = sparkMax.getEncoder();
            extensionEncoder.setPositionConversionFactor(EXTENSION_ROTATIONS_TO_METERS);
            extensionEncoder.setVelocityConversionFactor(EXTENSION_ROTATIONS_TO_METERS / 60.0);
            extensionEncoder.setPosition(0);
            
            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) (EXTENSION_LIMIT + Units.inchesToMeters(.5)));
            sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.inchesToMeters(-2));
            sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);

            extensionPidController = MotorUtil.createSparkMaxPIDController(sparkMax, extensionEncoder);
            extensionPidController.setP(extensionP);
            extensionPidController.setI(extensionI);
            extensionPidController.setD(extensionD);
            extensionPidController.setSmartMotionAllowedClosedLoopError(extensionTolerance, 0);
        });

        extensionFollow = MotorUtil.createSparkMax(EXTENSION_FOLLOW_ID, (sparkMax) -> {
            sparkMax.follow(extensionMotor);
            sparkMax.setIdleMode(IdleMode.kBrake);
        });

        if (Constants.IS_R1) extensionFollowB = null;
        else extensionFollowB = MotorUtil.createSparkMax(EXTENSION_FOLLOW_B_ID, (sparkMax) -> {
            sparkMax.follow(extensionMotor);
            sparkMax.setIdleMode(IdleMode.kBrake);
        });

        zeroLimitSwitch = new DigitalInput(ZERO_LIMIT_ID);

        if (Constants.IS_R1) leftHallSensor = null;
        else leftHallSensor = new HallEffectSensor(LEFT_HALL_ID, LEFT_MAGNETS, extensionEncoder.getPosition());

        shuffleboardTab = Shuffleboard.getTab("Tilted Elevator");

        extensionPEntry = shuffleboardTab.add("Extension P", extensionP).withPosition(0, 0).getEntry();
        extensionIEntry = shuffleboardTab.add("Extension I", extensionI).withPosition(1, 0).getEntry();
        extensionDEntry = shuffleboardTab.add("Extension D", extensionD).withPosition(2, 0).getEntry();
        extensionToleranceEntry = shuffleboardTab.add("Extension tolerance", extensionTolerance).withPosition(0, 1).getEntry();
        arbFFEntry = shuffleboardTab.add("Arb FF", arbFeedforward).withPosition(1, 1).getEntry();
        rampEntry = shuffleboardTab.add("Ramp Rate", extensionRampRate).withPosition(2, 1).getEntry();
        hallEntry = shuffleboardTab.add("Hall effect", hallPressed).getEntry();

        manualPowerEntry = shuffleboardTab.add("Manual Power", manualPower).withPosition(0, 2).getEntry();
        targetExtensionEntry = shuffleboardTab.add("Target Ext (in)", 0.0).withPosition(1, 2).getEntry();

        currentStateEntry = shuffleboardTab.add("Current state", state.toString()).withPosition(0, 3).getEntry();
        currentExtensionEntry = shuffleboardTab.add("Current Ext (in)", 0.0).withPosition(1, 3).getEntry();
        currentVelEntry = shuffleboardTab.add("Current Vel (mps)", 0.0).withPosition(2, 3).getEntry();
        offsetDistEntry = shuffleboardTab.add("Offset (in)", offsetDistMeters).withPosition(2, 2).getEntry();

        limitSwitchEntry = shuffleboardTab.add("Zero limit switch", false).withPosition(4, 0).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

        if (!Constants.IS_R1)
            leftHallSensor.addToShuffleboard(shuffleboardTab, 4, 1);

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
        // When limit switch is pressed, reset encoder
        if (zeroLimitSwitch != null && !zeroLimitSwitch.get())
            extensionEncoder.setPosition(0); 

        // When magnet is detected, reset encoder
        if (leftHallSensor != null) {
            HallEffectMagnet leftHallSensorPos = leftHallSensor.getHallEffectState(extensionEncoder.getPosition());
            // System.out.println(leftHallSensorPos);
            // System.out.println(lastHallPos + " " + leftHallSensorPos);
            if (leftHallSensorPos != null && lastHallPos == null){
                // extensionEncoder.setPosition(Units.inchesToMeters(EXTENSION_LIMIT));//leftHallSensorPos.getExtendDistanceMeters());
            }
            lastHallPos = leftHallSensorPos;
        }
        
        // If we're in manual power mode, use percent out power supplied by driver joystick.
        if (IS_MANUAL) {
            manualPowerEntry.setDouble(manualPower);
            extensionMotor.set(manualPower);
            return;
        }

        if (state == ElevatorState.HOME) {
            if (zeroLimitSwitch.get()) {
                extensionMotor.set(-0.25);
                extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
            } else {
                extensionMotor.set(0);
                extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
                state = ElevatorState.GROUND;
            }
            return;
        }

        extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        // Temporarily store mechanism state during single periodic loop
        double currentPos = extensionEncoder.getPosition();
        double currentVel = extensionEncoder.getVelocity();

        if (SHUFFLEBOARD_ENABLE) {
            ShuffleboardUtil.pollShuffleboardDouble(extensionPEntry, extensionPidController::setP);
            ShuffleboardUtil.pollShuffleboardDouble(extensionIEntry, extensionPidController::setI);
            ShuffleboardUtil.pollShuffleboardDouble(extensionDEntry, extensionPidController::setD);
            ShuffleboardUtil.pollShuffleboardDouble(extensionToleranceEntry, (value) -> extensionPidController.setSmartMotionAllowedClosedLoopError(value, 0));
            ShuffleboardUtil.pollShuffleboardDouble(rampEntry, (value) -> extensionMotor.setClosedLoopRampRate(value));
            arbFeedforward = arbFFEntry.getDouble(arbFeedforward);
        }

        // If we're trying to get to 0, set the motor to 0 power so the carriage drops with gravity
        // and hits the hard stop / limit switch.
        double targetExtension = getTargetExtension();
        if (targetExtension == 0 && currentPos < Units.inchesToMeters(1) && zeroLimitSwitch.get()) {
            extensionMotor.set(-0.075);
        }
        // If we're trying to get max extension and we're currently within 1'' of our goal, move elevator up so it hits the magnet
        // else if (targetExtension >= EXTENSION_LIMIT && (EXTENSION_LIMIT - currentPos < Units.inchesToMeters(1))) {
        //     extensionMotor.set(0.075);
        // } 
        // else {
            // Set PID reference
            extensionPidController.setReference(
                MathUtil.clamp(targetExtension, 0, EXTENSION_LIMIT + Units.inchesToMeters(.75)),
                ControlType.kPosition, 0,
                arbFeedforward, ArbFFUnits.kPercentOut
            );
        // }

        if (SHUFFLEBOARD_ENABLE) {
            currentExtensionEntry.setDouble(Units.metersToInches(currentPos));
            currentVelEntry.setDouble(currentVel);
            currentStateEntry.setString(state.toString());
            targetExtensionEntry.setDouble(Units.metersToInches(targetExtension));
            offsetDistEntry.setDouble(Units.metersToInches(offsetDistMeters));
            limitSwitchEntry.setBoolean(!zeroLimitSwitch.get());
            hallEntry.setBoolean(leftHallSensor.getHallEffectState(extensionEncoder.getPosition()) != null);
        }
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
    public double getExtension() { 
        return extensionEncoder.getPosition();
    }

    /**
     * Gets the target extension, in meters, of the subsystem.
     * @return The target extension, in meters.
     */
    public double getTargetExtension() {
        return state.getExtension(offsetState, this.pieceGrabbed) + offsetDistMeters;
    }

    /**
     * Gets whether the elevator has reached its target extension.
     * @return Whether the elevator has reached its target extension.
     */
    public boolean atTarget() {
        return Math.abs(getExtension() - getTargetExtension()) <= EXTENSION_TOLERANCE;
    }
}
