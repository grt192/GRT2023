package frc.robot.subsystems;

import static frc.robot.Constants.TiltedElevatorConstants.EXTENSION_FOLLOW_ID;
import static frc.robot.Constants.TiltedElevatorConstants.EXTENSION_ID;
import static frc.robot.Constants.TiltedElevatorConstants.EXTENSION_LIMIT;
import static frc.robot.Constants.TiltedElevatorConstants.EXTENSION_ROT_TO_M;
import static frc.robot.Constants.TiltedElevatorConstants.ZERO_LIMIT_ID;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;

public class TiltedElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax extensionMotor;
    private final RelativeEncoder extensionEncoder;
    private final SparkMaxPIDController extensionPidController;

    private final CANSparkMax extensionFollowMotor;

    private double extensionP = 0.25; //.4 // 4.9;
    private double extensionI = 0;
    private double extensionD = 0.65; //.2
    private double extensionFF = 0.3; //0.1

    private double maxVel = 0.5; // m/s
    private double maxAccel = 0.5; // 0.6 // m/s^2

    private ElevatorState currentState = ElevatorState.GROUND;

    private boolean IS_MANUAL = false;
    private double manualPower = 0;

    private double offsetDist = 0; // meters

    private final DigitalInput zeroLimitSwitch = new DigitalInput(ZERO_LIMIT_ID);

    // Shuffleboard
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Tilted Elevator");
    private final GenericEntry extensionPEntry = shuffleboardTab.add("Extension P", extensionP).getEntry();
    private final GenericEntry extensionIEntry = shuffleboardTab.add("Extension I", extensionI).getEntry();
    private final GenericEntry extensionDEntry = shuffleboardTab.add("Extension D", extensionD).getEntry();
    private final GenericEntry extensionFFEntry = shuffleboardTab.add("Extension FF", extensionFF).getEntry();

    private final GenericEntry manualPowerEntry = shuffleboardTab.add("Manual Power", manualPower).getEntry();

    private final GenericEntry maxVelEntry = shuffleboardTab.add("Max Vel", maxVel).getEntry();
    private final GenericEntry maxAccelEntry = shuffleboardTab.add("Max Accel", maxAccel).getEntry();

    private final GenericEntry currentVelEntry = shuffleboardTab.add("Current Vel (mps)", 0.0).getEntry();

    private final GenericEntry currentExtensionEntry = shuffleboardTab.add("Current Ext (in)", 0.0).getEntry();
    private final GenericEntry currentStateEntry = shuffleboardTab.add("Current State", currentState.toString())
            .getEntry();

    private final GenericEntry offsetDistEntry = shuffleboardTab.add("offset (in)", offsetDist).getEntry();

    private final GenericEntry targetExtensionEntry = shuffleboardTab.add("Target Ext (in)", 0.0).getEntry();
    private double targetExtension = 0;

    public enum ElevatorState {
        GROUND(0), // retracted
        CHUTE(Units.inchesToMeters(40)),
        SUBSTATION(Units.inchesToMeters(50)), // absolute height = 37.375 in
        CUBEMID(Units.inchesToMeters(33)), // absolute height = 14.25 in
        CUBEHIGH(Units.inchesToMeters(53)), // absolute height = 31.625 in
        CONEMID(Units.inchesToMeters(50)), // absolute height = 34 in
        CONEHIGH(Units.inchesToMeters(0)); // absolute height = 46 in

        public double extendDistance; // meters, extension distance of winch
        public double targetXDistance; // meters, x distance from middle of target (peg/shelf) to robot origin

        private double CUBE_OFFSET = Units.inchesToMeters(0); // meters, intake jaw to carriage bottom
        private double CONE_OFFSET = Units.inchesToMeters(0); // meters, intake jaw to carriage bottom

        /**
         * ElevatorState defined by absolute height of elevator from ground. All values
         * in meters and radians.
         * 
         * @param absoluteHeight ground to bottom of intake/outtake
         */
        private ElevatorState(double extendDistance) {
            this.extendDistance = extendDistance;
            this.targetXDistance = 0; // TODO
        }

        /**
         * Converts absolute height to extension distance.
         * 
         * @param absH absolute height, in meters
         * @return extension distance, in meters
         */
        private static double absHeightToExtendDist(double absHeight) {
            final double THETA = Math.toRadians(45.0);
            final double GND_TO_MECH_BOTTOM = Units.inchesToMeters(10.042344); // inches

            return (absHeight - GND_TO_MECH_BOTTOM) / Math.sin(THETA);
        }

    }

    public TiltedElevatorSubsystem() {
        extensionMotor = MotorUtil.createSparkMax(EXTENSION_ID);

        extensionMotor.setIdleMode(IdleMode.kBrake); 

        extensionMotor.setInverted(true); // flip

        extensionFollowMotor = MotorUtil.createSparkMax(EXTENSION_FOLLOW_ID);
        extensionFollowMotor.follow(extensionMotor);
        extensionFollowMotor.setIdleMode(IdleMode.kBrake);

        extensionEncoder = extensionMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(EXTENSION_ROT_TO_M);
        extensionEncoder.setVelocityConversionFactor(EXTENSION_ROT_TO_M / 60.0);
        extensionEncoder.setPosition(0);

        extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        extensionMotor.setSoftLimit(SoftLimitDirection.kForward, EXTENSION_LIMIT);
        extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.inchesToMeters(-2));

        extensionPidController = extensionMotor.getPIDController();
        extensionPidController.setP(extensionP);
        extensionPidController.setI(extensionI);
        extensionPidController.setD(extensionD);
        extensionPidController.setFF(extensionFF);
        extensionPidController.setSmartMotionMaxVelocity(maxVel, 0);
        extensionPidController.setSmartMotionMaxAccel(maxAccel, 0);
        extensionPidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    }

    @Override
    public void periodic() {

        if (zeroLimitSwitch.get()) {
            this.extensionEncoder.setPosition(0);
        }

        if (IS_MANUAL) {
            manualPowerEntry.setDouble(manualPower);
            extensionMotor.set(manualPower);
            return;
        }

        // Otherwise, use PID
        extensionPidController.setP(extensionPEntry.getDouble(extensionP));
        extensionPidController.setI(extensionIEntry.getDouble(extensionI));
        extensionPidController.setD(extensionDEntry.getDouble(extensionD));
        extensionPidController.setFF(extensionFFEntry.getDouble(extensionFF));
        extensionPidController.setSmartMotionMaxVelocity(maxVelEntry.getDouble(maxVel), 0);
        extensionPidController.setSmartMotionMaxAccel(maxAccelEntry.getDouble(maxAccel), 0);

        // System.out.println(extensionEncoder.getPosition());

        double currentPos = extensionEncoder.getPosition();
        double currentVel = extensionEncoder.getVelocity();

        currentVelEntry.setDouble(currentVel);
        currentExtensionEntry.setDouble(Units.metersToInches(currentPos));
        offsetDistEntry.setDouble(Units.metersToInches(offsetDist));

        // Units.inchesToMeters(targetExtensionEntry.getDouble(0));
        this.targetExtension = currentState.extendDistance + offsetDist;

        // give up 
        if (this.targetExtension == 0 && currentPos < Units.inchesToMeters(1)) {
            extensionMotor.set(0);
        } else if (this.targetExtension == 0 && currentPos < Units.inchesToMeters(5)) {
            // bring down to lim switch
            extensionMotor.set(-0.075);
        } else {
            // Set PID reference
            extensionPidController.setReference(MathUtil.clamp(targetExtension, 0, EXTENSION_LIMIT), ControlType.kSmartMotion);
        }

        currentStateEntry.setString(currentState.toString());
    }

    /**
     * Toggles between the two specified ElevatorStates and resets offset. Assign to
     * state1 by default.
     * 
     * @param state1 state 1
     * @param state2 state 2
     */
    public void toggleState(ElevatorState state1, ElevatorState state2) {
        resetOffset();

        currentState = currentState == state1
        ? state2
        : state1;
        
    }

    public void setOffsetDist(double power) {
        final double OFFSET_FACTOR = 0.01;
        this.offsetDist += OFFSET_FACTOR * power;
    }

    /**
     * Resets offset to 0.29.954896198482967
     */
    public void resetOffset() {
        this.offsetDist = 0;

    }

    /**
     * Set manual power.
     * 
     * @param power Xbox-controlled power
     */
    public void setManualPower(double manualPower) {

        if (manualPower > 0.5) {
            this.manualPower = 0.3;
        } else if (manualPower < -0.5) {
            this.manualPower = -0.3;
        } else {
            this.manualPower = 0;
        }
    }

}
