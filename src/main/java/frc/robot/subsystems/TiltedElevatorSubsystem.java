package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.util.MotorUtil;
import frc.robot.util.ShuffleboardUtil;
import frc.robot.motorcontrol.HallEffectSensor;
import frc.robot.motorcontrol.HallEffectMagnet;

import static frc.robot.Constants.TiltedElevatorConstants.*;

public class TiltedElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax extensionMotor;
    private RelativeEncoder extensionEncoder;
    private SparkMaxPIDController extensionPidController;

    private final CANSparkMax extensionFollow;
    private final CANSparkMax extensionFollowB;

    private final DigitalInput zeroLimitSwitch;
    private final HallEffectSensor leftHallSensor;

    private static final double EXTENSION_GEAR_RATIO = 14.0 / 64.0;
    private static final double EXTENSION_CIRCUMFERENCE = Units.inchesToMeters(Math.PI * 0.500); // approx circumference of winch
    private static final double EXTENSION_ROTATIONS_TO_METERS = EXTENSION_GEAR_RATIO * EXTENSION_CIRCUMFERENCE * 2.0 * (15.0 / 13.4);

    private static final double extensionP = 2.3;
    private static final double extensionI = 0;
    private static final double extensionD = 0;
    private static final double extensionTolerance = 0.003;
    private static final double extensionRampRate = 0.4;
    private double arbFeedforward = 0;

    private ElevatorState state = ElevatorState.GROUND;

    private static final boolean IS_MANUAL = false;
    private double manualPower = 0;

    private static final double OFFSET_FACTOR = 0.01; // The factor to multiply driver input by when changing the offset.
    private double offsetDistMeters = 0;

    public boolean pieceGrabbed = false;
    private boolean hallPressed = false;

    private final ShuffleboardTab shuffleboardTab;
    private final GenericEntry 
        extensionPEntry, extensionIEntry, extensionDEntry,
        extensionToleranceEntry, arbFFEntry, rampEntry;
    private final GenericEntry manualPowerEntry, targetExtensionEntry;
    private final GenericEntry currentExtensionEntry, currentVelEntry, currentStateEntry, offsetDistEntry;
    private final GenericEntry limitSwitchEntry, hallEntry;

    private HallEffectMagnet lastHallPos = null;


    public enum ElevatorState {
        GROUND(0) {
            @Override
            public double getExtension(boolean hasPiece) {
                // If a piece is being held, raise it slightly so that it doesn't drag across the floor.
                return hasPiece 
                    ? extendDistanceMeters + Units.inchesToMeters(5)
                    : extendDistanceMeters;
            }
        },
        CHUTE(Units.inchesToMeters(40)),
        SUBSTATION(Units.inchesToMeters(50)), // absolute height = 37.375 in
        CUBE_MID(Units.inchesToMeters(Constants.IS_R1 ? 33 : 40)), // absolute height = 14.25 in
        CUBE_HIGH(Units.inchesToMeters(Constants.IS_R1 ? 53 : 55)), // absolute height = 31.625 in
        CONE_MID(Units.inchesToMeters(48)), // absolute height = 34 in
        CONE_MID_DROP(CONE_MID.getExtension(false) - Units.inchesToMeters(10)),
        CONE_HIGH(Units.inchesToMeters(62.5)), // absolute height = 46 in
        CONE_HIGH_DROP(CONE_HIGH.getExtension(false) - Units.inchesToMeters(0)),
        HYBRID(Units.inchesToMeters(20)),//NEEDS TUNING
        HOME(Units.inchesToMeters(0));

        protected double extendDistanceMeters; // meters, extension distance of winch

        /**
         * ElevatorState defined by extension of elevator from zero. All values in meters and radians.
         * @param extendDistanceMeters The extension of the subsystem.
         */
        private ElevatorState(double extendDistanceMeters) {
            this.extendDistanceMeters = extendDistanceMeters;
        }

        /**
         * Gets the extension commanded by the elevator state.
         * @param hasPiece Whether there is a game piece in the subsystem.
         * @return The distance, in meters, the elevator should extend to.
         */
        public double getExtension(boolean hasPiece) {
            return this.extendDistanceMeters;
        }
    }

    public TiltedElevatorSubsystem() {
        extensionMotor = MotorUtil.createSparkMax(EXTENSION_ID, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake); 
            sparkMax.setInverted(true);
            sparkMax.setClosedLoopRampRate(extensionRampRate);

            extensionEncoder = sparkMax.getEncoder();
            extensionEncoder.setPositionConversionFactor(EXTENSION_ROTATIONS_TO_METERS);
            extensionEncoder.setVelocityConversionFactor(EXTENSION_ROTATIONS_TO_METERS / 60.0);
            extensionEncoder.setPosition(0);

            sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) (EXTENSION_LIMIT + Units.inchesToMeters(.5)));
            sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.inchesToMeters(-2));

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
                extensionEncoder.setPosition(Units.inchesToMeters(62.5));//leftHallSensorPos.getExtendDistanceMeters());
            }
            lastHallPos = leftHallSensorPos;
        }
        
        // If we're in manual power mode, use percent out power supplied by driver joystick.
        if (IS_MANUAL) {
            manualPowerEntry.setDouble(manualPower);
            extensionMotor.set(manualPower);
            return;
        }

        if(state == ElevatorState.HOME){
            if(zeroLimitSwitch != null && zeroLimitSwitch.get()){
                extensionMotor.set(-.1);
                extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
                return;
            } else if(!zeroLimitSwitch.get()){
                extensionMotor.set(0);
                extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
                state = ElevatorState.GROUND;
                return;
            } else {
                extensionMotor.set(0);
            }
        } else {
            extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        }
        
        // Temporarily store mechanism state during single periodic loop
        double currentPos = extensionEncoder.getPosition();
        double currentVel = extensionEncoder.getVelocity();

        ShuffleboardUtil.pollShuffleboardDouble(extensionPEntry, extensionPidController::setP);
        ShuffleboardUtil.pollShuffleboardDouble(extensionIEntry, extensionPidController::setI);
        ShuffleboardUtil.pollShuffleboardDouble(extensionDEntry, extensionPidController::setD);
        ShuffleboardUtil.pollShuffleboardDouble(extensionToleranceEntry, (value) -> extensionPidController.setSmartMotionAllowedClosedLoopError(value, 0));
        ShuffleboardUtil.pollShuffleboardDouble(rampEntry, (value) -> extensionMotor.setClosedLoopRampRate(value));
        arbFeedforward = arbFFEntry.getDouble(arbFeedforward);
        double targetExtension = getTargetExtension();


        // If we're trying to get to 0, set the motor to 0 power so the carriage drops with gravity
        // and hits the hard stop / limit switch.
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
                MathUtil.clamp(targetExtension, 0, EXTENSION_LIMIT),
                ControlType.kPosition, 0,
                arbFeedforward, ArbFFUnits.kPercentOut
            );
        // }

        currentExtensionEntry.setDouble(Units.metersToInches(currentPos));
        currentVelEntry.setDouble(currentVel);
        currentStateEntry.setString(state.toString());
        targetExtensionEntry.setDouble(Units.metersToInches(targetExtension));
        offsetDistEntry.setDouble(Units.metersToInches(offsetDistMeters));
        limitSwitchEntry.setBoolean(!zeroLimitSwitch.get());
        hallEntry.setBoolean(leftHallSensor.getHallEffectState(extensionEncoder.getPosition()) != null);
    }

    /**
     * Sets the state of the subsystem.
     * @param state The `ElevatorState` to set the subsystem to.
     */
    public void setState(ElevatorState state) {
        resetOffset();
        this.state = state;
    }

    /**
     * Gets the state of the subsystem.
     * @return The `ElevatorState` of the subsystem.
     */
    public ElevatorState getState() {
        return state;
    }

    /**
     * Toggles the state of the subsystem between the two specified `ElevatorState`s, defaulting
     * to state 1. Also resets the distance offset.
     * 
     * @param state1 The first state.
     * @param state2 The second state.
     */
    public void toggleState(ElevatorState state1, ElevatorState state2) {
        resetOffset();
        state = state == state1
            ? state2
            : state1;
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
        return state.getExtension(pieceGrabbed) + offsetDistMeters;
    }

    /**
     * Gets whether the elevator has reached its target extension.
     * @return Whether the elevator has reached its target extension.
     */
    public boolean atTarget() {
        return Math.abs(getExtension() - getTargetExtension()) <= EXTENSION_TOLERANCE;
    }
}
