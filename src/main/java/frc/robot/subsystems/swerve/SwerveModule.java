package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import frc.robot.motorcontrol.MotorUtil;

/**
 * A swerve module with a Falcon drive motor and a NEO steer motor.
 */
public class SwerveModule {
    // private final WPI_TalonFX driveMotor;
    private final CANSparkMax driveMotor;

    private final CANSparkMax steerMotor;
    private final RelativeEncoder steerEncoder;
    private final SparkMaxAnalogSensor steerAbsoluteEncoder;
    private final SparkMaxPIDController steerPidController;

    private final double offsetRads;

    private static final double DRIVE_ROTATIONS_TO_METERS = (1.0 / 3.0) * (13.0 / 8.0) * (1.0 / 3.0) * Math.PI * Units.inchesToMeters(4.0);
    private static final double STEER_ROTATIONS_TO_RADIANS = (1.0 / 52.0) * (34.0 / 63.0) * 2 * Math.PI; // 52:1 gear ratio, 63:34 pulley ratio, 1 rotation = 2pi
    private static final double STEER_VOLTS_TO_RADIANS = 2 * Math.PI / 3.3; // MA3 analog output: 3.3V -> 2pi

    private static final double driveP = 0;
    private static final double driveI = 0;
    private static final double driveD = 0;
    private static final double driveFF = 0;

    private static final double steerP = 0.4;
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
        /*
        driveMotor = MotorUtil.createTalonFX(drivePort);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setSensorPhase(false);
        driveMotor.config_kP(0, driveP);
        driveMotor.config_kI(0, driveI);
        driveMotor.config_kD(0, driveD);
        driveMotor.config_kF(0, driveFF);
        */

        driveMotor = MotorUtil.createSparkMax(drivePort);
        driveMotor.setIdleMode(IdleMode.kBrake);

        steerMotor = MotorUtil.createSparkMax(steerPort);
        steerMotor.setIdleMode(IdleMode.kBrake);

        steerAbsoluteEncoder = steerMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        steerAbsoluteEncoder.setPositionConversionFactor(STEER_VOLTS_TO_RADIANS);

        steerEncoder = steerMotor.getEncoder();
        steerEncoder.setPositionConversionFactor(STEER_ROTATIONS_TO_RADIANS);
        steerEncoder.setPosition(steerAbsoluteEncoder.getPosition()); // Set initial position to absolute value

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
            // driveMotor.getSelectedSensorVelocity() * DRIVE_TICKS_TO_METERS * 10.0,
            driveMotor.get(),
            getAngle()
        );
    }

    /**
     * Sets the desired state of the module.
     * @param state The desired state of the module as a `SwerveModuleState`.
     */
    public void setDesiredState(SwerveModuleState state) {
        var optimized = optimizeModuleState(state, getAngle());
        // driveMotor.set(ControlMode.Velocity, optimized.getFirst() / (DRIVE_TICKS_TO_METERS * 10.0));
        driveMotor.set(optimized.getFirst()); // Only while the module is in percent output
        steerPidController.setReference(optimized.getSecond() - offsetRads, ControlType.kPosition);
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

    /**
     * Returns the current angle of the module. This differs from the raw encoder reading 
     * because this applies `offsetRads` to zero the module at a desired angle.
     * 
     * @return The current angle of the module, as a `Rotation2d`.
     */
    private Rotation2d getAngle() {
        return new Rotation2d(steerEncoder.getPosition() + offsetRads);
    }

    /**
     * Utility class to construct a top left swerve module from a module's pin offset.
     * The offset to align the pin with the front of the robot is automatically applied.
     */
    public static class TopLeft extends SwerveModule {
        public TopLeft(int drivePort, int steerPort) {
            super(drivePort, steerPort, 0.0);
        }
        public TopLeft(int drivePort, int steerPort, double offsetRads) {
            super(drivePort, steerPort, offsetRads);
        }
    }

    /**
     * Utility class to construct a top right swerve module from a module's pin offset.
     * The offset to align the pin with the front of the robot is automatically applied.
     */
    public static class TopRight extends SwerveModule {
        public TopRight(int drivePort, int steerPort) {
            super(drivePort, steerPort, -Math.PI / 2.0);
        }
        public TopRight(int drivePort, int steerPort, double offsetRads) {
            super(drivePort, steerPort, offsetRads - Math.PI / 2.0);
        }
    }

    /**
     * Utility class to construct a bottom left swerve module from a module's pin offset.
     * The offset to align the pin with the front of the robot is automatically applied.
     */
    public static class BottomLeft extends SwerveModule {
        public BottomLeft(int drivePort, int steerPort) {
            super(drivePort, steerPort, Math.PI / 2.0);
        }
        public BottomLeft(int drivePort, int steerPort, double offsetRads) {
            super(drivePort, steerPort, offsetRads + Math.PI / 2.0);
        }
    }

    /**
     * Utility class to construct a bottom right swerve module from a module's pin offset.
     * The offset to align the pin with the front of the robot is automatically applied.
     */
    public static class BottomRight extends SwerveModule {
        public BottomRight(int drivePort, int steerPort) {
            super(drivePort, steerPort, Math.PI);
        }
        public BottomRight(int drivePort, int steerPort, double offsetRads) {
            super(drivePort, steerPort, offsetRads + Math.PI);
        }
    }
}
