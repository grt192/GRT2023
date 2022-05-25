package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A swerve module with a Falcon drive motor and a NEO steer motor.
 */
public class SwerveModule {
    private final WPI_TalonFX driveMotor;

    private final CANSparkMax steerMotor;
    private final RelativeEncoder steerEncoder;
    private final SparkMaxPIDController steerPidController;

    private final double offsetRads;

    private static final double DRIVE_TICKS_TO_METERS = 1.0;

    private static final double driveP = 0;
    private static final double driveI = 0;
    private static final double driveD = 0;
    private static final double driveFF = 0;

    private static final double steerP = 0;
    private static final double steerI = 0;
    private static final double steerD = 0;
    private static final double steerFF = 0;

    /**
     * Constructs a SwerveModule from a drive and steer motor CAN ID and an angle offset.
     * The offset will be applied to all angle readings to change the zero point of the 
     * module.
     * 
     * @param drivePort The drive TalonFX CAN ID.
     * @param steerPort The steer SparkMax CAN ID.
     * @param offsetRads The angle offset, in radians.
     */
    public SwerveModule(int drivePort, int steerPort, double offsetRads) {
        driveMotor = new WPI_TalonFX(drivePort);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setSensorPhase(false);
        driveMotor.config_kP(0, driveP);
        driveMotor.config_kI(0, driveI);
        driveMotor.config_kD(0, driveD);
        driveMotor.config_kF(0, driveFF);

        steerMotor = new CANSparkMax(steerPort, MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.setIdleMode(IdleMode.kBrake);

        steerEncoder = steerMotor.getAlternateEncoder(1024);
        steerEncoder.setPositionConversionFactor(2 * Math.PI); // 1 rotation = 2pi

        steerPidController = steerMotor.getPIDController();
        steerPidController.setP(steerP);
        steerPidController.setI(steerI);
        steerPidController.setD(steerD);
        steerPidController.setFF(steerFF);

        this.offsetRads = offsetRads;
    }

    /**
     * Constructs a SwerveModule from a drive and steer motor CAN ID, 
     * defaulting the angle offset to 0.
     * 
     * @param drivePort The drive TalonFX CAN ID.
     * @param steerPort The steer SparkMax CAN ID.
     */
    public SwerveModule(int drivePort, int steerPort) {
        this(drivePort, steerPort, 0.0);
    }

    /**
     * Gets the current state of the module as a `SwerveModuleState`.
     * @return The state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getSelectedSensorVelocity() * DRIVE_TICKS_TO_METERS * 10.0, // u/100ms -> m/s 
            getAngle()
        );
    }

    /**
     * Sets the desired state of the module.
     * @param state The desired state of the module as a `SwerveModuleState`.
     */
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        driveMotor.set(ControlMode.Velocity, optimized.speedMetersPerSecond / DRIVE_TICKS_TO_METERS / 10.0); // m/s -> u/100ms

        // Wrap current angle between [0, 2pi]
        double currentAngle = getAngle().getRadians();
        double wrappedAngle = MathUtil.inputModulus(currentAngle, 0, 2 * Math.PI);

        // If the delta angle is greater than 180, go the other way by wrapping angle between [-pi, pi]
        double deltaRads = MathUtil.angleModulus(optimized.angle.getRadians() - wrappedAngle);

        steerPidController.setReference(currentAngle + deltaRads, ControlType.kPosition);
    }

    /**
     * Returns the current angle of the module.
     * @return The current angle of the module, as a `Rotation2d`.
     */
    private Rotation2d getAngle() {
        return new Rotation2d(steerEncoder.getPosition() + offsetRads);
    }
}
