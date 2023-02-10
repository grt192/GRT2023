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

public class PivotElevatorSubsystem extends SubsystemBase{
    private final CANSparkMax rotationMotor;
    private RelativeEncoder rotationEncoder;
    private SparkMaxPIDController rotationPidController;

    private final CANSparkMax rotationMotorFollower;

    private final CANSparkMax extensionMotor;
    private RelativeEncoder extensionEncoder;
    private SparkMaxPIDController extensionPidController;

    private double rotationP = 0.125;
    private double rotationI = 0;
    private double rotationD = 0;
    private final double ROTATION_ROT_TO_RAD = ROTATION_GEAR_RATIO * 2 * Math.PI;

    private double extensionP = 0.125;
    private double extensionI = 0;
    private double extensionD = 0;
    private final double EXTENSION_ROT_TO_M = Units.inchesToMeters(0.5615); 

    private final boolean RESET_OFFSET_ON_STAGE_SWITCH = true;
    private double angleOffset = 0; //radians
    private double extensionOffset = 0; //meters

    private MoverPosition currentState = MoverPosition.VERTICAL;

    private final boolean TESTING = true;

    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Mover Subsystem");
    private final GenericEntry currentAngleEntry = shuffleboardTab.add("current angle",  0.0).getEntry();
    private final GenericEntry currentExtensionEntry = shuffleboardTab.add("current extension", 0.0).getEntry();
    private final GenericEntry targetAngleEntry = shuffleboardTab.add("target angle", 0.0).getEntry();
    private final GenericEntry targetExtensionEntry = shuffleboardTab.add("target extension", 0.0).getEntry();

    private final GenericEntry rotationPEntry = shuffleboardTab.add("Rotation P", rotationP).getEntry();
    private final GenericEntry rotationIEntry = shuffleboardTab.add("Rotation I", rotationI).getEntry();
    private final GenericEntry rotationDEntry = shuffleboardTab.add("Rotation D", rotationD).getEntry();

    private final GenericEntry extensionPEntry = shuffleboardTab.add("Extension P", extensionP).getEntry();
    private final GenericEntry extensionIEntry = shuffleboardTab.add("Extension I", extensionI).getEntry();
    private final GenericEntry extensionDEntry = shuffleboardTab.add("Extension D", extensionD).getEntry();


    public enum GamePiece {
        CONE, CUBE;
    }
    
    public enum MoverPosition {
        VERTICAL(0,0),
        GROUND(Units.degreesToRadians(90), distanceToExtension(Units.inchesToMeters(25))),
        SUBSTATION(Units.degreesToRadians(65), distanceToExtension(Units.inchesToMeters(44))),
        CUBEMID(Units.degreesToRadians(72), distanceToExtension(Units.inchesToMeters(34))), CUBEHIGH(Units.degreesToRadians(66),distanceToExtension(Units.inchesToMeters(44))), 
        CONEMID(Units.degreesToRadians(62),distanceToExtension(Units.inchesToMeters(34))), CONEHIGH(Units.degreesToRadians(66),distanceToExtension(Units.inchesToMeters(44)))
        ;

        public double angle; //radians, 0 is vertical
        public double extension; //meters, 0 extension is 25 inches from pivot

        private MoverPosition(double angle, double extension){
            this.angle = angle;
            this.extension = extension;
        }
    }

    public PivotElevatorSubsystem(){
        rotationMotor = MotorUtil.createSparkMax(ROTATION_MOTOR_PORT, (CANSparkMax sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake);
            sparkMax.setInverted(true);

            rotationEncoder = sparkMax.getEncoder();
            rotationEncoder.setPositionConversionFactor(ROTATION_ROT_TO_RAD);
            rotationEncoder.setPosition(0);
    
            rotationPidController = sparkMax.getPIDController();
            rotationPidController.setP(rotationP);
            rotationPidController.setI(rotationI);
            rotationPidController.setD(rotationD);
            
            sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
            sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) Units.degreesToRadians(90));
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.degreesToRadians(-90));
        });
        rotationMotorFollower = MotorUtil.createSparkMax(ROTATION_FOLLOWER_MOTOR_PORT, (CANSparkMax sparkMax) -> {
            sparkMax.follow(rotationMotor);
        });

        extensionMotor = MotorUtil.createSparkMax(EXTENSION_MOTOR_PORT, (CANSparkMax sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake);
            

            extensionEncoder = sparkMax.getEncoder();
            extensionEncoder.setPositionConversionFactor(EXTENSION_ROT_TO_M);
            extensionEncoder.setPosition(0);
            
            sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
            sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) Units.inchesToMeters(24));
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.inchesToMeters(0));

            extensionPidController = sparkMax.getPIDController();
            extensionPidController.setP(extensionP);
            extensionPidController.setI(extensionI);
            extensionPidController.setD(extensionD);
        });

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
        
        double rotp = rotationPEntry.getDouble(rotationP);
        double roti = rotationIEntry.getDouble(rotationI);
        double rotd = rotationDEntry.getDouble(rotationD);

        double extp = extensionPEntry.getDouble(extensionP);
        double exti = extensionIEntry.getDouble(extensionI);
        double extd = extensionDEntry.getDouble(extensionD);


        if(rotp != rotationP){
            rotationP = rotp;
        }
        if(roti != rotationI){
            rotationI = roti;
        }
        if(rotd != rotationD){
            rotationD = rotd;
        }
        if(extp != extensionP){
            extensionP = extp;
        }
        if(exti != extensionI){
            extensionI = exti;
        }
        if(extd != extensionD){
            extensionD = extd;
        }

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
