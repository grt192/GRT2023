package frc.robot.subsystems.drivetrain;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Constants;
import frc.robot.util.MotorUtil;
import frc.robot.util.ShuffleboardUtil;

/**
 * A swerve module with a Falcon drive motor and a NEO steer motor.
 */
public class SwerveModule implements BaseSwerveModule {
    // private final WPI_TalonFX driveMotor;
    private final CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkMaxPIDController drivePidController;

    private final CANSparkMax steerMotor;
    private RelativeEncoder steerRelativeEncoder;
    private SparkMaxAnalogSensor steerAbsoluteEncoder;
    private SparkMaxPIDController steerPidController;

    private final double offsetRads;
    private boolean relativeFeedbackEnabled = false;

    private static final double DRIVE_ROTATIONS_TO_METERS = (1.0 / 3.0) * (13.0 / 8.0) * (1.0 / 3.0) * Math.PI * Units.inchesToMeters(4.0) * 9.0 / 9.5; // 3:1, 8:13, 3:1 gear ratios, 4.0" wheel diameter, circumference = pi * d
    private static final double STEER_ROTATIONS_TO_RADIANS = (1.0 / 52.0) * (34.0 / 63.0) * 2 * Math.PI; // 52:1 gear ratio, 63:34 pulley ratio, 1 rotation = 2pi
    private static final double STEER_VOLTS_TO_RADIANS = 2 * Math.PI / 3.3; // MA3 analog output: 3.3V -> 2pi

    private static final double driveP = 0.05;
    private static final double driveI = 0;
    private static final double driveD = 0;
    private static final double driveFF = 0.176697057706;

    private static final double steerP = 0.7;
    private static final double steerI = 0;
    private static final double steerD = 0;
    private static final double steerFF = 0;

    private final ShuffleboardTab shuffleboardTab;
    private final GenericEntry 
        targetVelEntry, currentVelEntry, velErrorEntry,
        targetAngleEntry, currentAngleEntry, angleErrorEntry;

    // Whether to read and update shuffleboard values
    private static final boolean OVERRIDE_SHUFFLEBOARD_ENABLE = false;
    private volatile boolean SHUFFLEBOARD_ENABLE = OVERRIDE_SHUFFLEBOARD_ENABLE || Constants.GLOBAL_SHUFFLEBOARD_ENABLE;

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
         * driveMotor = MotorUtil.createTalonFX(drivePort);
         * driveMotor.setNeutralMode(NeutralMode.Brake);
         * 
         * driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
         * driveMotor.setSensorPhase(false);
         * driveMotor.config_kP(0, driveP);
         * driveMotor.config_kI(0, driveI);
         * driveMotor.config_kD(0, driveD);
         * driveMotor.config_kF(0, driveFF);
         */

        driveMotor = MotorUtil.createSparkMax(drivePort, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake);

            driveEncoder = sparkMax.getEncoder();
            driveEncoder.setPositionConversionFactor(DRIVE_ROTATIONS_TO_METERS);
            driveEncoder.setVelocityConversionFactor(DRIVE_ROTATIONS_TO_METERS / 60.0); // min = 60s
    
            drivePidController = MotorUtil.createSparkMaxPIDController(sparkMax, driveEncoder);
            drivePidController.setP(driveP);
            drivePidController.setI(driveI);
            drivePidController.setD(driveD);
            drivePidController.setFF(driveFF);
        });

        steerMotor = MotorUtil.createSparkMax550(steerPort, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake);

            steerAbsoluteEncoder = sparkMax.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
            steerAbsoluteEncoder.setPositionConversionFactor(STEER_VOLTS_TO_RADIANS);

            steerRelativeEncoder = sparkMax.getEncoder();
            steerRelativeEncoder.setPositionConversionFactor(STEER_ROTATIONS_TO_RADIANS);
            steerRelativeEncoder.setPosition(steerAbsoluteEncoder.getPosition()); // Set initial position to absolute value

            steerPidController = MotorUtil.createSparkMaxPIDController(sparkMax, steerAbsoluteEncoder);
            steerPidController.setP(steerP);
            steerPidController.setI(steerI);
            steerPidController.setD(steerD);
            steerPidController.setFF(steerFF);
    
            steerPidController.setPositionPIDWrappingEnabled(true);
            steerPidController.setPositionPIDWrappingMinInput(0.0);
            steerPidController.setPositionPIDWrappingMaxInput(2 * Math.PI);
        });

        shuffleboardTab = Shuffleboard.getTab("Swerve " + drivePort + " " + steerPort);
        targetVelEntry = shuffleboardTab.add("Target velocity (mps)", 0.0)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
        currentVelEntry = shuffleboardTab.add("Current velocity (mps)", 0.0)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
        velErrorEntry = shuffleboardTab.add("Velocity error (mps)", 0.0)
            .withPosition(0, 1)
            .withSize(5, 3)
            // .withWidget(BuiltInWidgets.kGraph)
            .getEntry();

        targetAngleEntry = shuffleboardTab.add("Target angle (degs)", 0.0)
            .withPosition(6, 0)
            .withSize(2, 1)
            .getEntry();
        currentAngleEntry = shuffleboardTab.add("Current angle (degs)", 0.0)
            .withPosition(8, 0)
            .withSize(2, 1)
            .getEntry();
        angleErrorEntry = shuffleboardTab.add("Angle error (degs)", 0.0)
            .withPosition(6, 1)
            .withSize(5, 3)
            // .withWidget(BuiltInWidgets.kGraph)
            .getEntry();

        GenericEntry shuffleboardEnableEntry = shuffleboardTab.add("Shuffleboard enable", SHUFFLEBOARD_ENABLE)
            .withPosition(5, 0)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
        ShuffleboardUtil.addBooleanListener(shuffleboardEnableEntry, (value) -> SHUFFLEBOARD_ENABLE = value);

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
     * Gets the current state of the module as a `SwerveModulePosition`.
     * @return The state of the module.
     */
    public SwerveModulePosition getState() {
        return new SwerveModulePosition(
            // driveMotor.getSelectedSensorPosition() * DRIVE_TICKS_TO_METERS,
            driveEncoder.getPosition(),
            getWrappedAngle()
        );
    }

    /**
     * Sets the desired state of the module.
     * @param state The desired state of the module as a `SwerveModuleState`.
     */
    public void setDesiredState(SwerveModuleState state) {
        // If we're using the absolute encoder, keep all angles wrapped and rely on PID wrapping for
        // the setpoint. Otherwise, use the unwrapped angle and optimize with wraparound.
        Rotation2d currentAngle = relativeFeedbackEnabled ? getRelativeAngle() : getWrappedAngle();
        SwerveModuleState optimized = relativeFeedbackEnabled
            ? optimizeWithWraparound(state, currentAngle)
            : SwerveModuleState.optimize(state, currentAngle);

        double targetAngleRads = optimized.angle.getRadians() - offsetRads;
        double angleErrorRads = optimized.angle.minus(currentAngle).getRadians();

        double currentVelocity = driveEncoder.getVelocity();
        double targetVelocity = optimized.speedMetersPerSecond * Math.abs(Math.cos(angleErrorRads));

        // Set shuffleboard debug info
        if (SHUFFLEBOARD_ENABLE) {
            targetVelEntry.setDouble(targetVelocity);
            currentVelEntry.setDouble(currentVelocity);
            velErrorEntry.setDouble(targetVelocity - currentVelocity);

            targetAngleEntry.setDouble(Math.toDegrees(MathUtil.angleModulus(targetAngleRads + offsetRads)));
            currentAngleEntry.setDouble(currentAngle.getDegrees());
            angleErrorEntry.setDouble(Math.toDegrees(angleErrorRads));
        }

        // driveMotor.set(ControlMode.Velocity, optimized.getFirst() / (DRIVE_TICKS_TO_METERS * 10.0));
        drivePidController.setReference(targetVelocity, ControlType.kVelocity);
        steerPidController.setReference(targetAngleRads, ControlType.kPosition);
    }

    /**
     * Optimizes a `SwerveModuleState` by inverting the wheel speeds and rotating the other direction
     * if the delta angle is greater than 90 degrees. This method also handles angle wraparound.
     * 
     * @param target The target `SwerveModuleState`.
     * @param currentUnwrappedAngle The current *unwrapped* angle of the module, as a `Rotation2d`.
     * @return The optimized `SwerveModuleState`.
     */
    public static SwerveModuleState optimizeWithWraparound(SwerveModuleState target, Rotation2d currentUnwrappedAngle) {
        double angleRads = currentUnwrappedAngle.getRadians();

        double targetVel = target.speedMetersPerSecond;
        double targetWrappedAngleRads = target.angle.getRadians();
        double deltaRads = MathUtil.angleModulus(targetWrappedAngleRads - angleRads);

        // Optimize the `SwerveModuleState` if delta angle > 90 by flipping wheel speeds
        // and going the other way.
        if (Math.abs(deltaRads) > Math.PI / 2.0) {
            targetVel = -targetVel;
            deltaRads += deltaRads > Math.PI / 2.0 ? -Math.PI : Math.PI;
        }

        return new SwerveModuleState(targetVel, new Rotation2d(angleRads + deltaRads));
    }

    @Override
    public void setSteerRelativeFeedback(boolean useRelative) {
        steerPidController.setFeedbackDevice(useRelative ? steerRelativeEncoder : steerAbsoluteEncoder);
        steerPidController.setPositionPIDWrappingEnabled(!useRelative);

        this.relativeFeedbackEnabled = useRelative;
    }

    /**
     * Returns the current (wrapped) angle of the module. This differs from the raw encoder reading
     * because this applies `offsetRads` to zero the module at a desired angle.
     * 
     * @return The current [-pi, pi] angle of the module, as a `Rotation2d`.
     */
    private Rotation2d getWrappedAngle() {
        double angleRads = relativeFeedbackEnabled
            ? steerRelativeEncoder.getPosition()
            : steerAbsoluteEncoder.getPosition();
        double wrappedAngleRads = MathUtil.angleModulus(angleRads + offsetRads);

        return new Rotation2d(wrappedAngleRads);
    }

    /**
     * Gets the angle of the module reported by the relative encoder. This applies `offsetRads`, but is not wrapped.
     * @return The current unwrapped angle of the module, as a `Rotation2d`.
     */
    private Rotation2d getRelativeAngle() {
        double angleRads = steerRelativeEncoder.getPosition() + offsetRads;
        return new Rotation2d(angleRads);
    }

    /**
     * Returns the raw value, in radians, reported by the steer absolute encoder.
     * @return The current radians reported by the absolute encoder.
     */
    public double getAbsoluteRawAngleRads() {
        return steerAbsoluteEncoder.getPosition();
    }

    /**
     * Returns the position offset, in radians, used to align the zero of the module with the front
     * of the robot (ex. for the top right module, this is -pi/2).
     * 
     * @return The position offset, in radians.
     */
    public double getPositionOffsetRads() {
        return 0.0;
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
        private static final double POS_OFFSET = -Math.PI / 2.0;

        public TopRight(int drivePort, int steerPort) {
            super(drivePort, steerPort, POS_OFFSET);
        }

        public TopRight(int drivePort, int steerPort, double offsetRads) {
            super(drivePort, steerPort, offsetRads + POS_OFFSET);
        }

        @Override
        public double getPositionOffsetRads() {
            return POS_OFFSET;
        }
    }

    /**
     * Utility class to construct a bottom left swerve module from a module's pin offset.
     * The offset to align the pin with the front of the robot is automatically applied.
     */
    public static class BottomLeft extends SwerveModule {
        private static final double POS_OFFSET = Math.PI / 2.0;

        public BottomLeft(int drivePort, int steerPort) {
            super(drivePort, steerPort, POS_OFFSET);
        }

        public BottomLeft(int drivePort, int steerPort, double offsetRads) {
            super(drivePort, steerPort, offsetRads + POS_OFFSET);
        }

        @Override
        public double getPositionOffsetRads() {
            return POS_OFFSET;
        }
    }

    /**
     * Utility class to construct a bottom right swerve module from a module's pin offset.
     * The offset to align the pin with the front of the robot is automatically applied.
     */
    public static class BottomRight extends SwerveModule {
        private static final double POS_OFFSET = Math.PI;

        public BottomRight(int drivePort, int steerPort) {
            super(drivePort, steerPort, POS_OFFSET);
        }

        public BottomRight(int drivePort, int steerPort, double offsetRads) {
            super(drivePort, steerPort, offsetRads + POS_OFFSET);
        }

        @Override
        public double getPositionOffsetRads() {
            return POS_OFFSET;
        }
    }
}
