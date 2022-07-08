package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.motorcontrol.GRTTalonFX;

/**
 * A swerve module with a Falcon drive motor and a NEO steer motor.
 */
public class SwerveModule {
    private final GRTTalonFX driveMotor;

    private final CANSparkMax steerMotor;
    private final SparkMaxAnalogSensor steerEncoder;
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
        driveMotor = new GRTTalonFX(drivePort);
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setSensorPhase(false);
        driveMotor.setVelocityConversionFactor(DRIVE_TICKS_TO_METERS * 10.0); // u/100ms -> m/s 
        driveMotor.config_kP(0, driveP);
        driveMotor.config_kI(0, driveI);
        driveMotor.config_kD(0, driveD);
        driveMotor.config_kF(0, driveFF);

        steerMotor = new CANSparkMax(steerPort, MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.setIdleMode(IdleMode.kBrake);

        steerEncoder = steerMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        steerEncoder.setPositionConversionFactor(2 * Math.PI); // 1 rotation = 2pi

        steerPidController = steerMotor.getPIDController();
        steerPidController.setFeedbackDevice(steerEncoder);
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
            driveMotor.getSelectedSensorVelocity(),
            getAngle()
        );
    }

    /**
     * Sets the desired state of the module.
     * @param state The desired state of the module as a `SwerveModuleState`.
     */
    public void setDesiredState(SwerveModuleState state) {
        double realAngle = getAngle().getRadians();
        double wrappedAngle = MathUtil.angleModulus(realAngle);

        double targetVel = state.speedMetersPerSecond;
        double targetWrappedAngle = state.angle.getRadians();
        double deltaRads = wrappedAngle - targetWrappedAngle;

        // Optimize the `SwerveModuleState` if delta angle > 90 by flipping wheel speeds
        // and going the other way.
        if (Math.abs(deltaRads) > Math.PI / 2.0) {
            targetVel = -targetVel;
            targetWrappedAngle += deltaRads > Math.PI / 2.0 ? -Math.PI : Math.PI;
        }

        driveMotor.set(ControlMode.Velocity, targetVel);
        steerPidController.setReference(realAngle - offsetRads + deltaRads, ControlType.kPosition);
    }

    /**
     * Returns the current angle of the module. This differs from the raw encoder reading 
     * because this applies `offsetRads` to zero the module at a desired angle.
     * 
     * @return The current angle of the module, as a `Rotation2d`.
     */
    private Rotation2d getAngle() {
        return new Rotation2d(steerEncoder.getPosition() + offsetRads);
    }
}
