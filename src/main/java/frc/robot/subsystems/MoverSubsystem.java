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
import frc.robot.motorcontrol.MotorUtil;
import static frc.robot.Constants.MoverConstants.*;

public class MoverSubsystem extends SubsystemBase{
    private final CANSparkMax rotationMotor;
    private final RelativeEncoder rotationEncoder;
    private final SparkMaxPIDController rotationPidController;

    private final CANSparkMax rotationMotorFollower;

    private final CANSparkMax extensionMotor;
    private final RelativeEncoder extensionEncoder;
    private final SparkMaxPIDController extensionPidController;

    private final double ROTATION_P = 0.125;
    private final double ROTATION_I = 0;
    private final double ROTATION_D = 0;
    private final double ROTATION_ROT_TO_RAD = ROTATION_GEAR_RATIO * 2 * Math.PI;

    private final double EXTENSION_P = 0.125;
    private final double EXTENSION_I = 0;
    private final double EXTENSION_D = 0;
    private final double EXTENSION_ROT_TO_M = Units.inchesToMeters(0.5615); 

    private final boolean RESET_OFFSET_ON_STAGE_SWITCH = true;
    private double angleOffset = 0; //radians
    private double extensionOffset = 0; //meters

    private MoverPosition currentState = MoverPosition.GROUND;

    private final boolean TESTING = true;

    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Mover Subsystem");
    private final GenericEntry currentAngleEntry = shuffleboardTab.add("current angle",  0.0).getEntry();
    private final GenericEntry currentExtensionEntry = shuffleboardTab.add("current extension", 0.0).getEntry();
    private final GenericEntry targetAngleEntry = shuffleboardTab.add("target angle", 0.0).getEntry();
    private final GenericEntry targetExtensionEntry = shuffleboardTab.add("target extension", 0.0).getEntry();


    public enum GamePiece {
        CONE, CUBE;
    }
    
    public enum MoverPosition {
        VERTICAL(0,0),
        GROUND(Units.degreesToRadians(90), distanceToExtension(25)),
        SUBSTATION(0,0),
        CUBEMID(0,0), CUBEHIGH(0,0), 
        CONEMID(0,0), CONEHIGH(0,0)
        ;

        public double angle; //radians, 0 is vertical
        public double extension; //meters, 0 extension is 25 inches from pivot

        private MoverPosition(double angle, double extension){
            this.angle = angle;
            this.extension = extension;
        }
    }

    public MoverSubsystem(){
        rotationMotor = MotorUtil.createSparkMax(ROTATION_MOTOR_PORT);
        rotationMotor.setIdleMode(IdleMode.kBrake);
        rotationMotorFollower = MotorUtil.createSparkMax(ROTATION_FOLLOWER_MOTOR_PORT);
        rotationMotorFollower.follow(rotationMotor);

        rotationEncoder = rotationMotor.getEncoder();
        rotationEncoder.setPositionConversionFactor(ROTATION_ROT_TO_RAD);
        
        rotationMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        rotationMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rotationMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Units.degreesToRadians(90));
        rotationMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.degreesToRadians(-90));

        rotationPidController = rotationMotor.getPIDController();
        rotationPidController.setP(ROTATION_P);
        rotationPidController.setI(ROTATION_I);
        rotationPidController.setD(ROTATION_D);


        extensionMotor = MotorUtil.createSparkMax(EXTENSION_MOTOR_PORT);
        extensionMotor.setIdleMode(IdleMode.kBrake);

        extensionEncoder = extensionMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(EXTENSION_ROT_TO_M);
        
        extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        extensionMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Units.inchesToMeters(29));
        extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.inchesToMeters(0));

        extensionPidController = extensionMotor.getPIDController();
        extensionPidController.setP(EXTENSION_P);
        extensionPidController.setI(EXTENSION_I);
        extensionPidController.setD(EXTENSION_D);

    }

    @Override
    public void periodic(){
        if(!TESTING){
            goTo(currentState.angle + angleOffset, currentState.extension + extensionOffset);
        }

        currentAngleEntry.setDouble(Units.radiansToDegrees(rotationEncoder.getPosition()));
        currentExtensionEntry.setDouble(Units.metersToInches(extensionEncoder.getPosition()));
        targetAngleEntry.setDouble(Units.radiansToDegrees(currentState.angle + angleOffset));
        targetExtensionEntry.setDouble(Units.metersToInches(currentState.extension + extensionOffset));
    }


    private void goTo(double angle, double extension){
        rotationPidController.setReference(angle, ControlType.kPosition);
        extensionPidController.setReference(extension, ControlType.kPosition);
    }

    public void setState(MoverPosition state){
        currentState = state;
        if(RESET_OFFSET_ON_STAGE_SWITCH){
            angleOffset = 0;
            extensionOffset = 0;
        }
    }

    public MoverPosition getState(){
        return currentState;
    }

    private static double distanceToExtension(double distance){
        return distance - Units.inchesToMeters(25);
    }

    public void setPowers(double xPower, double yPower){
        if(TESTING){
            setMotorPowers(xPower, yPower);
        } else {
            setOffsetPowers(xPower, yPower);
        }
    }
    public void setOffsetPowers(double xPower, double yPower){
        
        angleOffset += xPower * ANGLE_OFFSET_SPEED;
        extensionOffset += yPower * EXTENSION_OFFSET_SPEED;
    }

    public void setMotorPowers(double xPower, double yPower){
        rotationMotor.set(xPower * .2);
        extensionMotor.set(yPower * .2);
    }

}
