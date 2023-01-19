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

import frc.robot.motorcontrol.MotorUtil;

/**
 * A swerve module with a Falcon drive motor and a NEO steer motor.
 */
public class SwerveModule implements BaseSwerveModule {
    // private final WPI_TalonFX driveMotor;
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePidController;

    private final CANSparkMax steerMotor;
    private final SparkMaxAnalogSensor steerAbsoluteEncoder;
    private final SparkMaxPIDController steerPidController;

    private final double offsetRads;

    private static final double DRIVE_ROTATIONS_TO_METERS = (1.0 / 3.0) * (13.0 / 8.0) * (1.0 / 3.0) * Math.PI * Units.inchesToMeters(4.0); // 3:1, 8:13, 3:1 gear ratios, 4.0" wheel diameter, circumference = pi * d
    private static final double STEER_VOLTS_TO_RADIANS = 2 * Math.PI / 3.3; // MA3 analog output: 3.3V -> 2pi

    private static final double driveP = 0;
    private static final double driveI = 0;
    private static final double driveD = 0;
    private static final double driveFF = 0.176697057706;

    private static final double steerP = 0.6;
    private static final double steerI = 0;
    private static final double steerD = 0;
    private static final double steerFF = 0;

    private final ShuffleboardTab shuffleboardTab;
    private final GenericEntry 
        targetVelEntry, currentVelEntry, velErrorEntry,
        targetAngleEntry, currentAngleEntry, angleErrorEntry;

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

        driveMotor = MotorUtil.createSparkMax(drivePort);
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(DRIVE_ROTATIONS_TO_METERS);
        driveEncoder.setVelocityConversionFactor(DRIVE_ROTATIONS_TO_METERS / 60.0); // min = 60s

        drivePidController = driveMotor.getPIDController();
        drivePidController.setP(driveP);
        drivePidController.setI(driveI);
        drivePidController.setD(driveD);
        drivePidController.setFF(driveFF);

        steerMotor = MotorUtil.createSparkMax(steerPort);
        steerMotor.setIdleMode(IdleMode.kBrake);

        steerAbsoluteEncoder = steerMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        steerAbsoluteEncoder.setPositionConversionFactor(STEER_VOLTS_TO_RADIANS);

        steerPidController = steerMotor.getPIDController();
        steerPidController.setFeedbackDevice(steerAbsoluteEncoder);
        steerPidController.setP(steerP);
        steerPidController.setI(steerI);
        steerPidController.setD(steerD);
        steerPidController.setFF(steerFF);

        steerPidController.setPositionPIDWrappingEnabled(true);
        steerPidController.setPositionPIDWrappingMinInput(0.0);
        steerPidController.setPositionPIDWrappingMaxInput(2 * Math.PI);

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
        angleErrorEntry = shuffleboardTab.add("Angle error (rads)", 0.0)
            .withPosition(6, 1)
            .withSize(5, 3)
            // .withWidget(BuiltInWidgets.kGraph)
            .getEntry();

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
            getAngle()
        );
    }

    /**
     * Sets the desired state of the module.
     * @param state The desired state of the module as a `SwerveModuleState`.
     */
    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentAngle = getAngle();
        SwerveModuleState optimized = SwerveModuleState.optimize(state, currentAngle);

        double currentVelocity = driveEncoder.getVelocity();
        double targetAngle = optimized.angle.getRadians() - offsetRads;

        // Set shuffleboard debug info
        targetVelEntry.setDouble(optimized.speedMetersPerSecond);
        currentVelEntry.setDouble(currentVelocity);
        velErrorEntry.setDouble(optimized.speedMetersPerSecond - currentVelocity);

        targetAngleEntry.setDouble(Math.toDegrees(MathUtil.angleModulus(targetAngle)));
        currentAngleEntry.setDouble(currentAngle.minus(new Rotation2d(offsetRads)).getDegrees());
        angleErrorEntry.setDouble(MathUtil.angleModulus(targetAngle - currentAngle.getRadians() + offsetRads));

        // driveMotor.set(ControlMode.Velocity, optimized.getFirst() / (DRIVE_TICKS_TO_METERS * 10.0));
        drivePidController.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity);
        // driveMotor.set(optimized.speedMetersPerSecond);
        steerPidController.setReference(targetAngle, ControlType.kPosition);
    }

    /**
     * Returns the current (wrapped) angle of the module. This differs from the raw encoder reading
     * because this applies `offsetRads` to zero the module at a desired angle.
     * 
     * @return The current [-pi, pi] angle of the module, as a `Rotation2d`.
     */
    private Rotation2d getAngle() {
        // System.out.println(Units.feetToMeters(16.10) * 0.3);
        // System.out.println(driveEncoder.getVelocity());
        // System.out.println(steerAbsoluteEncoder.getPosition());

        double wrappedAngleRads = MathUtil.angleModulus(steerAbsoluteEncoder.getPosition() + offsetRads);
        return new Rotation2d(wrappedAngleRads);
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
