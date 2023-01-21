package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TiltedElevatorConstants.*;
import frc.robot.motorcontrol.MotorUtil;

public class TiltedElevatorSubsystem  extends SubsystemBase {

    private final CANSparkMax extensionMotor;
    private final RelativeEncoder extensionEncoder;
    private final SparkMaxPIDController extensionPidController;

    private double extensionP = 0.125;
    private double extensionI = 0;
    private double extensionD = 0;

    private ElevatorState currentState = ElevatorState.GROUND;

    public final static double OFFSET_FACTOR = 0.00001; // meters
    private double offsetDist = 0;

    // Shuffleboard
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Tilted Elevator");
    private final GenericEntry extensionPEntry = shuffleboardTab.add("Extension P", extensionP).getEntry();
    private final GenericEntry extensionIEntry = shuffleboardTab.add("Extension I", extensionI).getEntry();
    private final GenericEntry extensionDEntry = shuffleboardTab.add("Extension D", extensionD).getEntry();

    private final GenericEntry currentExtensionEntry = shuffleboardTab.add("Current Extension", 0.0).getEntry();
    private final GenericEntry currentStateEntry = shuffleboardTab.add("Current State", currentState.toString()).getEntry();

    public enum ElevatorState {
        GROUND(0), // retracted
        SUBSTATION(Units.inchesToMeters(38.71)), // absolute height = 37.375 in
        CUBEMID(Units.inchesToMeters(6.01)),  // absolute height = 14.25 in
        CUBEHIGH(Units.inchesToMeters(30.58)),  // absolute height = 31.625 in
        CONEMID(Units.inchesToMeters(33.94)),  // absolute height = 34 in
        CONEHIGH(Units.inchesToMeters(50.91)); // absolute height = 46 in

        public double extendDistance; // meters, extension distance of winch
        public double targetXDistance; // meters, x distance from middle of target (peg/shelf) to robot origin

        private double CUBE_OFFSET = Units.inchesToMeters(0); // meters, intake jaw to carriage bottom
        private double CONE_OFFSET = Units.inchesToMeters(0); // meters, intake jaw to carriage bottom

        /**
         * ElevatorState defined by absolute height of elevator from ground. All values in meters and radians.
         * @param absoluteHeight ground to bottom of intake/outtake
         */
        private ElevatorState(double extendDistance) {
            this.extendDistance = extendDistance;
            this.targetXDistance = 0; // TODO
        }

        /**
         * Converts absolute height to extension distance.
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
        extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        extensionMotor.setSoftLimit(SoftLimitDirection.kForward, EXTENSION_LIMIT);
        extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, -1);
        extensionMotor.setIdleMode(IdleMode.kBrake);

        extensionEncoder = extensionMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(EXTENSION_ROT_TO_M);
        extensionEncoder.setPosition(0);

        extensionPidController = extensionMotor.getPIDController();
        extensionPidController.setP(extensionP);
        extensionPidController.setI(extensionI);
        extensionPidController.setD(extensionD);

    }

    @Override
    public void periodic() {

        // Verify extension distance
        double finalExtendDist = currentState.extendDistance + offsetDist;
        if (finalExtendDist >= 0 && finalExtendDist <= EXTENSION_LIMIT) {
            extensionPidController.setReference(finalExtendDist, ControlType.kPosition);
        }

        // Shuffleboard
        extensionP = extensionPEntry.getDouble(extensionP);
        extensionI = extensionIEntry.getDouble(extensionI);
        extensionD = extensionDEntry.getDouble(extensionD);

        currentExtensionEntry.setDouble(Units.metersToInches(extensionEncoder.getPosition() + offsetDist));
        currentStateEntry.setString(currentState.toString());
    }

    /**
     * Toggles between the two specified ElevatorStates and resets offset. Assign to state1 by default.
     * @param state1 state 1
     * @param state2 state 2
     */
    public void toggleState(ElevatorState state1, ElevatorState state2) { 
        resetOffset();

        if (currentState == state1) {
            currentState = state2; // toggle
        } else if (currentState == state2) {
            currentState = state1; // toggle
        } else {
            currentState = state1; // assign state1 by default
        }
    }

    /**
     * Resets offset to 0.
     */
    public void resetOffset() {
        this.offsetDist = 0;
    }

    /**
     * Set offset distance based on power.
     * @param power Xbox-controlled power
     */
    public void setOffsetPowers(double power) {
        this.offsetDist += OFFSET_FACTOR * power;
    }
    
}
