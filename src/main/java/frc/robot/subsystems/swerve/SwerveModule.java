package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePidController;

    private final CANSparkMax steerMotor;
    private final RelativeEncoder steerEncoder;
    private final SparkMaxPIDController steerPidController;

    private static final double DRIVE_ROTATIONS_TO_METERS = 1.0;
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
        driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setVelocityConversionFactor(DRIVE_ROTATIONS_TO_METERS / 60); // RPM -> m/s

        drivePidController = driveMotor.getPIDController();
        drivePidController.setP(driveP);
        drivePidController.setI(driveI);
        drivePidController.setD(driveD);
        drivePidController.setFF(driveFF);

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
            driveEncoder.getVelocity(), 
            getAngle()
        );
    }

    /**
     * Sets the desired state of the module.
     * @param state The desired state of the module as a `SwerveModuleState`.
     */
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        drivePidController.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity);
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
