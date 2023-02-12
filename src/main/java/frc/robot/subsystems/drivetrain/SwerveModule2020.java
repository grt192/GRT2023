package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.motorcontrol.MotorUtil;

/**
 * A swerve module with a NEO drive motor and a BAG steer motor, for running
 * swerve modules on the 2020 robot.
 */
public class SwerveModule2020 implements BaseSwerveModule {
    private final CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkMaxPIDController drivePidController;

    private final WPI_TalonSRX steerMotor;

    private final double offsetRads;

    private static final double DRIVE_ROTATIONS_TO_METERS = 0.04518592;
    private static final double STEER_TICKS_TO_RADIANS = 2 * Math.PI / 1024.0;

    private static final double driveP = 0;
    private static final double driveI = 0;
    private static final double driveD = 0;
    private static final double driveFF = 0;

    private static final double steerP = 5.5;
    private static final double steerI = 0;
    private static final double steerD = 0;
    private static final double steerFF = 0;

    public SwerveModule2020(int drivePort, int steerPort, double offsetRads) {
        driveMotor = MotorUtil.createSparkMax(drivePort, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake);

            driveEncoder = sparkMax.getEncoder();
            driveEncoder.setPositionConversionFactor(DRIVE_ROTATIONS_TO_METERS);
            driveEncoder.setVelocityConversionFactor(DRIVE_ROTATIONS_TO_METERS / 60.0); // RPM -> m/s

            drivePidController = MotorUtil.createSparkMaxPIDController(sparkMax, driveEncoder);
            drivePidController.setP(driveP);
            drivePidController.setI(driveI);
            drivePidController.setD(driveD);
            drivePidController.setFF(driveFF);
        });

        steerMotor = MotorUtil.createTalonSRX(steerPort);
        steerMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setInverted(true);

        steerMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerMotor.setSensorPhase(false);
        steerMotor.config_kP(0, steerP);
        steerMotor.config_kI(0, steerI);
        steerMotor.config_kD(0, steerD);
        steerMotor.config_kF(0, steerFF);
        steerMotor.configPeakOutputForward(0.65);
        steerMotor.configPeakOutputReverse(-0.65);

        this.offsetRads = offsetRads;
    }

    public SwerveModule2020(int drivePort, int steerPort) {
        this(drivePort, steerPort, 0.0);
    }

    /**
     * Gets the current state of the module as a `SwerveModuleState`.
     * @return The state of the module.
     */
    public SwerveModulePosition getState() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), // NOTE: this is only while the wheel velocity is in percent output instead
            getAngle()
        );
    }

    /**
     * Sets the desired state of the module.
     * @param state The desired state of the module as a `SwerveModuleState`.
     */
    public void setDesiredState(SwerveModuleState state) {
        var optimized = optimizeModuleState(state, getAngle());
        driveMotor.set(optimized.getFirst()); // NOTE: this is only while the wheel velocity is in percent output instead
        steerMotor.set(ControlMode.Position, (optimized.getSecond() - offsetRads) / STEER_TICKS_TO_RADIANS);
    }

    /**
     * Returns the current angle of the module. This differs from the raw encoder reading
     * because this applies `offsetRads` to zero the module at a desired angle.
     * 
     * @return The current angle of the module, as a `Rotation2d`.
     */
    private Rotation2d getAngle() {
        return new Rotation2d(
                steerMotor.getSelectedSensorPosition() * STEER_TICKS_TO_RADIANS + offsetRads);
    }

    /**
     * Optimizes a `SwerveModuleState` by inverting the wheel speeds and rotating the other direction
     * if the delta angle is greater than 90 degrees. This method also handles angle wraparound.
     * 
     * @param target The target `SwerveModuleState`.
     * @param currentAngle The current angle of the module, as a `Rotation2d`.
     * @return A pair representing [target velocity, target angle]. Note that `offsetRads` will still need to be applied before PID.
     */
    public static Pair<Double, Double> optimizeModuleState(SwerveModuleState target, Rotation2d currentAngle) {
        double angleRads = currentAngle.getRadians();

        double targetVel = target.speedMetersPerSecond;
        double targetWrappedAngle = target.angle.getRadians();
        double deltaRads = MathUtil.angleModulus(targetWrappedAngle - angleRads);

        // Optimize the `SwerveModuleState` if delta angle > 90 by flipping wheel speeds
        // and going the other way.
        if (Math.abs(deltaRads) > Math.PI / 2.0) {
            targetVel = -targetVel;
            deltaRads += deltaRads > Math.PI / 2.0 ? -Math.PI : Math.PI;
        }

        return new Pair<>(targetVel, angleRads + deltaRads);
    }
}
