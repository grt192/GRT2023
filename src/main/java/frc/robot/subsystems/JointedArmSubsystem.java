package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;

import static frc.robot.Constants.JointedArmConstants.*;

public class JointedArmSubsystem extends SubsystemBase {

    // Elevator motor
    private final CANSparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private final SparkMaxPIDController elevatorPidController;
    
    private final double ELEVATOR_TICKS_TO_METERS = Units.inchesToMeters(0.5615);

    private final double ELEVATOR_P = 0.125;
    private final double ELEVATOR_I = 0;
    private final double ELEVATOR_D = 0;

    // Shoulder pfft
    private final Solenoid shoulderPfft;

    // Elbow motors
    private final CANSparkMax elbowMotor;
    private final RelativeEncoder elbowEncoder;
    private final SparkMaxPIDController elbowPidController;
    private final CANSparkMax elbowFollowerMotor;

    private final double ELBOW_TICKS_TO_RADIANS = ELBOW_GEAR_RATIO * 2 * Math.PI / 42.0;

    private final double ELBOW_P = 0.125;
    private final double ELBOW_I = 0;
    private final double ELBOW_D = 0;

    // State vars
    private MoverPosition currentPosition = MoverPosition.RETRACTED;

    public enum MoverPosition {
        RETRACTED(0, false, 0),
        GROUND(0, false, Units.degreesToRadians(90)),
        SUBSTATION(0, true, 0),
        CUBEMID(0, true, 0), 
        CUBEHIGH(0, true, 0), 
        CONEMID(0, true, 0), 
        CONEHIGH(0, true, 0);

        public double elevatorHeightExt; // in meters
        public boolean shoulderExt; // extended or retracted
        public double elbowAngle; // in radians, angle relative to forearm, rotate out

        private MoverPosition(double elevatorHeightExt, boolean shoulderExt, double elbowAngle){
            this.elevatorHeightExt = elevatorHeightExt;
            this.shoulderExt = shoulderExt;
            this.elbowAngle = elbowAngle;
        }
    }


    public JointedArmSubsystem() {
        
        // Initialize elevator motor
        elevatorMotor = MotorUtil.createSparkMax(ELEVATOR_MOTOR_PORT);
        elevatorMotor.setIdleMode(IdleMode.kBrake);

        elevatorPidController = elevatorMotor.getPIDController();
        elevatorPidController.setP(ELEVATOR_P);
        elevatorPidController.setI(ELEVATOR_I);
        elevatorPidController.setD(ELEVATOR_D);

        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPositionConversionFactor(ELEVATOR_TICKS_TO_METERS);

        // Initialize shoulder pfft
        shoulderPfft = new Solenoid(PneumaticsModuleType.CTREPCM, SHOULDER_PFFT_PORT);

        // Initialize elbow motors
        elbowMotor = MotorUtil.createSparkMax(ELBOW_MOTOR_PORT);
        elbowMotor.setIdleMode(IdleMode.kBrake);

        elbowFollowerMotor = MotorUtil.createSparkMax(ELBOW_FOLLOWER_MOTOR_PORT);
        elbowFollowerMotor.follow(elbowMotor);

        elbowPidController = elbowMotor.getPIDController();
        elbowPidController.setP(ELBOW_P);
        elbowPidController.setI(ELBOW_I);
        elbowPidController.setD(ELBOW_D);

        elbowEncoder = elbowMotor.getEncoder();
        elbowEncoder.setPositionConversionFactor(ELBOW_TICKS_TO_RADIANS);

    }

    @Override
    public void periodic() {

        elevatorPidController.setReference(currentPosition.elevatorHeightExt, CANSparkMax.ControlType.kPosition);
        shoulderPfft.set(currentPosition.shoulderExt);
        elbowPidController.setReference(currentPosition.elbowAngle, CANSparkMax.ControlType.kPosition);

    }
    
}
