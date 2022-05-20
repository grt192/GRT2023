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

    private static final double DRIVE_TICKS_TO_METERS = 1.0;
    private static final double STEER_ROTATIONS_TO_RADIANS = 1.0;

    private static final double driveP = 0;
    private static final double driveI = 0;
    private static final double driveD = 0;
    private static final double driveFF = 0;

    private static final double steerP = 0;
    private static final double steerI = 0;
    private static final double steerD = 0;
    private static final double steerFF = 0;

    public SwerveModule(int drivePort, int steerPort) {
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

        steerEncoder = steerMotor.getAlternateEncoder(4096);
        steerEncoder.setPositionConversionFactor(STEER_ROTATIONS_TO_RADIANS);

        steerPidController = steerMotor.getPIDController();
        steerPidController.setP(steerP);
        steerPidController.setI(steerI);
        steerPidController.setD(steerD);
        steerPidController.setFF(steerFF);
    }

    /**
     * Gets the current state of the module as a `SwerveModuleState`.
     * @return The state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getSelectedSensorVelocity() * DRIVE_TICKS_TO_METERS * 10, // u/100ms -> m/s 
            getAngle()
        );
    }

    /**
     * Sets the desired state of the module.
     * @param state The desired state of the module as a `SwerveModuleState`.
     */
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        driveMotor.set(ControlMode.Velocity, optimized.speedMetersPerSecond / DRIVE_TICKS_TO_METERS / 10); // m/s -> u/100ms
        steerPidController.setReference(optimized.angle.getRadians(), ControlType.kSmartMotion);
    }

    /**
     * Returns the current angle of the module.
     * @return The current angle of the module, as a `Rotation2d`.
     */
    private Rotation2d getAngle() {
        return new Rotation2d(steerEncoder.getPosition());
    }
}
