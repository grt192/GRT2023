package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.motorcontrol.MotorUtil;

import static frc.robot.Constants.MoverConstants.*;

public class MoverSubsystem extends SubsystemBase{
    // private final DigitalInput crimitswitch;

    private final CANSparkMax rotationMotor;
    private final RelativeEncoder rotationEncoder;
    private final SparkMaxPIDController rotationPidController;

    private final CANSparkMax rotationMotorFollower;

    private final CANSparkMax extensionMotor;
    private final RelativeEncoder extensionEncoder;
    private final SparkMaxPIDController extensionPidController;

    private double rotationP = 0.125;
    private double rotationI = 0;
    private double rotationD = 0;
    private double rotationFF = 0;
    private double rotationMaxAccel = 0;
    private double rotationMaxVel = 0;
    private static final double ROTATION_ROTATIONS_TO_RADIANS = ROTATION_GEAR_RATIO * 2 * Math.PI;

    private double extensionP = 0.125;
    private double extensionI = 0;
    private double extensionD = 0;
    private double extensionFF = 0;
    private double extensionMaxAccel = 0;
    private double extensionMaxVel = 0;
    private static final double EXTENSION_ROTATIONS_TO_METERS = Units.inchesToMeters(0.5615); 

    private double angleOffsetRads = 0; //radians
    private double extensionOffsetMeters = 0; //meters
    private static final boolean RESET_OFFSET_ON_STAGE_SWITCH = true;

    private MoverPosition currentState = MoverPosition.VERTICAL;

    private static final boolean TESTING = false;

    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Mover Subsystem");
    private final GenericEntry currentAngleEntry = shuffleboardTab.add("current angle",  0.0).getEntry();
    private final GenericEntry currentExtensionEntry = shuffleboardTab.add("current extension", 0.0).getEntry();
    private final GenericEntry targetAngleEntry = shuffleboardTab.add("target angle", 0.0).getEntry();
    private final GenericEntry targetExtensionEntry = shuffleboardTab.add("target extension", 0.0).getEntry();

    private final GenericEntry rotationPEntry = shuffleboardTab.add("Rotation P", rotationP).getEntry();
    private final GenericEntry rotationIEntry = shuffleboardTab.add("Rotation I", rotationI).getEntry();
    private final GenericEntry rotationDEntry = shuffleboardTab.add("Rotation D", rotationD).getEntry();
    private final GenericEntry rotationFFEntry = shuffleboardTab.add("Rotation FF", rotationFF).getEntry();

    private final GenericEntry extensionPEntry = shuffleboardTab.add("Extension P", extensionP).getEntry();
    private final GenericEntry extensionIEntry = shuffleboardTab.add("Extension I", extensionI).getEntry();
    private final GenericEntry extensionDEntry = shuffleboardTab.add("Extension D", extensionD).getEntry();
    private final GenericEntry extensionFFEntry = shuffleboardTab.add("Extension FF", extensionFF).getEntry();

    public enum GamePiece {
        CONE, CUBE;
    }

    public enum MoverPosition {
        VERTICAL(0, 0),
        GROUND(
            Units.degreesToRadians(90), 
            distanceToExtension(Units.inchesToMeters(25))
        ),
        SUBSTATION(
            Units.degreesToRadians(65),
            distanceToExtension(Units.inchesToMeters(44))
        ),
        CUBEMID(
            Units.degreesToRadians(72),
            distanceToExtension(Units.inchesToMeters(34))
        ),
        CUBEHIGH(
            Units.degreesToRadians(66),
            distanceToExtension(Units.inchesToMeters(44))
        ), 
        CONEMID(
            Units.degreesToRadians(62),
            distanceToExtension(Units.inchesToMeters(34))
        ),
        CONEHIGH(
            Units.degreesToRadians(66),
            distanceToExtension(Units.inchesToMeters(44))
        );

        public double angleRads; // radians, 0 is vertical
        public double extensionMeters; // meters, 0 extension is 25 inches from pivot

        private MoverPosition(double angleRads, double extensionMeters) {
            this.angleRads = angleRads;
            this.extensionMeters = extensionMeters;
        }
    }

    public MoverSubsystem(){
        // crimitswitch = new DigitalInput(0);

        rotationMotor = MotorUtil.createSparkMax(ROTATION_MOTOR_PORT);
        rotationMotor.setIdleMode(IdleMode.kBrake);
        rotationMotorFollower = MotorUtil.createSparkMax(ROTATION_FOLLOWER_MOTOR_PORT);
        rotationMotorFollower.follow(rotationMotor);
        rotationMotorFollower.setIdleMode(IdleMode.kBrake);

        rotationEncoder = rotationMotor.getEncoder();
        rotationEncoder.setPositionConversionFactor(ROTATION_ROTATIONS_TO_RADIANS);
        rotationEncoder.setVelocityConversionFactor(ROTATION_ROTATIONS_TO_RADIANS / 60);
        rotationEncoder.setPosition(0);
        
        rotationMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        rotationMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rotationMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Units.degreesToRadians(90));
        rotationMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.degreesToRadians(-90));

        rotationPidController = rotationMotor.getPIDController();
        rotationPidController.setP(rotationP);
        rotationPidController.setI(rotationI);
        rotationPidController.setD(rotationD);
        rotationPidController.setFF(rotationFF);
        rotationPidController.setSmartMotionMaxVelocity(rotationMaxVel, 0);
        rotationPidController.setSmartMotionMaxAccel(rotationMaxAccel, 0);
        rotationPidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

        extensionMotor = MotorUtil.createSparkMax(EXTENSION_MOTOR_PORT);
        extensionMotor.setIdleMode(IdleMode.kBrake);

        extensionEncoder = extensionMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(EXTENSION_ROTATIONS_TO_METERS);
        extensionEncoder.setPositionConversionFactor(EXTENSION_ROTATIONS_TO_METERS / 60);
        extensionEncoder.setPosition(0);

        extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        extensionMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Units.inchesToMeters(24));
        extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.inchesToMeters(0));

        extensionPidController = extensionMotor.getPIDController();
        extensionPidController.setP(extensionP);
        extensionPidController.setI(extensionI);
        extensionPidController.setD(extensionD);
        extensionPidController.setFF(extensionFF);
        extensionPidController.setSmartMotionMaxVelocity(extensionMaxVel, 0);
        extensionPidController.setSmartMotionMaxAccel(extensionMaxAccel, 0);
        extensionPidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    }

    @Override
    public void periodic(){
        // if (!crimitswitch.get()){
        //     extensionEncoder.setPosition(0);
        // }

        // if (extensionEncoder.getPosition() <= 0 && crimitswitch.get()){
        //     extensionMotor.set(-.2);
        // }

        if (!TESTING) {
            double offsetAngleRads = currentState.angleRads + angleOffsetRads;
            double offsetExtensionMeters = currentState.extensionMeters + extensionOffsetMeters;

            rotationPidController.setReference(offsetAngleRads, ControlType.kSmartMotion);
            extensionPidController.setReference(offsetExtensionMeters, ControlType.kPosition);
        }

        currentAngleEntry.setDouble(Units.radiansToDegrees(rotationEncoder.getPosition()));
        currentExtensionEntry.setDouble(Units.metersToInches(extensionEncoder.getPosition()));
        targetAngleEntry.setDouble(Units.radiansToDegrees(currentState.angleRads + angleOffsetRads));
        targetExtensionEntry.setDouble(Units.metersToInches(currentState.extensionMeters + extensionOffsetMeters));

        double rotp = rotationPEntry.getDouble(rotationP);
        double roti = rotationIEntry.getDouble(rotationI);
        double rotd = rotationDEntry.getDouble(rotationD);
        double rotff = rotationFFEntry.getDouble(rotationFF);

        double extp = extensionPEntry.getDouble(extensionP);
        double exti = extensionIEntry.getDouble(extensionI);
        double extd = extensionDEntry.getDouble(extensionD);
        double extff = rotationFFEntry.getDouble(rotationFF);

        if (rotp != rotationP){
            rotationP = rotp;
            rotationPidController.setP(rotationP);
        }
        if (roti != rotationI){
            rotationI = roti;
            rotationPidController.setI(rotationI);
        }
        if (rotd != rotationD){
            rotationD = rotd;
            rotationPidController.setD(rotationD);
        }
        if (rotff != rotationFF){
            rotationFF = rotff;
            rotationPidController.setFF(rotationFF);
        }

        if (extp != extensionP){
            extensionP = extp;
            extensionPidController.setP(extensionP);
        }
        if (exti != extensionI){
            extensionI = exti;
            extensionPidController.setI(extensionI);
        }
        if (extd != extensionD){
            extensionD = extd;
            extensionPidController.setD(extensionD);
        }
        if (extff != extensionFF){
            extensionFF = extff;
            rotationPidController.setFF(extensionFF);
        }
    }

    /**
     * Sets the state of the subsystem.
     * @param state The `MoverPosition` state to set the subsystem to.
     */
    public void setState(MoverPosition state) {
        currentState = state;
        if (RESET_OFFSET_ON_STAGE_SWITCH) {
            angleOffsetRads = 0;
            extensionOffsetMeters = 0;
        }
    }

    /**
     * Gets the state of the subsystem.
     * @return Gets the `MoverPosition` state of the subsystem.
     */
    public MoverPosition getState() {
        return currentState;
    }

    private static double distanceToExtension(double distance) {
        return distance - Units.inchesToMeters(25);
    }

    public void setPowers(double xPower, double yPower) {
        if (TESTING) {
            setMotorPowers(xPower, yPower);
        } else {
            setOffsetPowers(xPower, yPower);
        }
    }

    public void setOffsetPowers(double xPower, double yPower){
        angleOffsetRads += xPower * ANGLE_OFFSET_SPEED;
        extensionOffsetMeters += yPower * EXTENSION_OFFSET_SPEED;
    }

    public void setMotorPowers(double xPower, double yPower){
        rotationMotor.set(xPower * .2);
        extensionMotor.set(yPower * .2);
    }
}
