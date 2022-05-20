package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.EntryNotification;

import frc.robot.shuffleboard.GRTShuffleboardTab;

/**
 * A swerve module with a NEO drive motor and a BAG steer motor, for running swerve modules on 
 * the 2020 robot.
 */
public class NEOTalonSwerveModule {
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePidController;

    private final WPI_TalonSRX steerMotor;

    private static final double DRIVE_ROTATIONS_TO_METERS = 1.0;
    private static final double STEER_TICKS_TO_RADIANS = Math.PI / 488.0;

    private static final double driveP = 0;
    private static final double driveI = 0;
    private static final double driveD = 0;
    private static final double driveFF = 0;

    private static final double steerP = 0.125;
    private static final double steerI = 0;
    private static final double steerD = 0;
    private static final double steerFF = 0;

    private final GRTShuffleboardTab shuffleboardTab = new GRTShuffleboardTab("Swerve");

    public NEOTalonSwerveModule(int drivePort, int steerPort) {
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

        steerMotor = new WPI_TalonSRX(steerPort);
        steerMotor.configFactoryDefault();
        steerMotor.setNeutralMode(NeutralMode.Brake);

        steerMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerMotor.setSensorPhase(false);
        steerMotor.config_kP(0, steerP);
        steerMotor.config_kI(0, steerI);
        steerMotor.config_kD(0, steerD);
        steerMotor.config_kF(0, steerFF);

        shuffleboardTab
            .list("Drive PID")
            .at(1, 0)
            .withSize(1, 3)
            .addListener("kP", driveP, this::setDriveP)
            .addListener("kI", driveI, this::setDriveI)
            .addListener("kD", driveD, this::setDriveD)
            .addListener("kFF", driveFF, this::setDriveFF);

        shuffleboardTab
            .list("Steer PID")
            .at(2, 0)
            .withSize(1, 3)
            .addListener("kP", steerP, this::setSteerP)
            .addListener("kI", steerI, this::setSteerI)
            .addListener("kD", steerD, this::setSteerD)
            .addListener("kFF", steerFF, this::setSteerFF);

        shuffleboardTab.addListener("Drive reference", 0, this::setDriveReference, 1, 3);
    }

    /**
     * Testing function to get the current velocity of the drive motor.
     * @return The current velocity of the drive motor, in RPM.
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Testing function to get the current position of the steer motor.
     * @return The current position of the steer motor, in encoder ticks.
     */
    public double getSteerPosition() {
        return steerMotor.getSelectedSensorPosition();
    }

    /**
     * Testing function to get the current velocity of the steer motor, in radians/second.
     * This relies on a correctly tuned STEER_TICKS_TO_RADIANS to return the correct units.
     * @return The current position of the steer motor, in radians/s.
     */
    public double getSteerVelocity() {
        // ticks/100ms -> rads/s
        return steerMotor.getSelectedSensorVelocity() * 10 * STEER_TICKS_TO_RADIANS;
    }

    /**
     * Gets the current state of the module as a `SwerveModuleState`.
     * @return The state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.get(), 
            getAngle()
        );
    }

    /**
     * Sets the desired state of the module.
     * @param state The desired state of the module as a `SwerveModuleState`.
     */
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        driveMotor.set(optimized.speedMetersPerSecond);
        steerMotor.set(ControlMode.MotionMagic, optimized.angle.getRadians() / STEER_TICKS_TO_RADIANS);
    }

    /**
     * Returns the current angle of the module.
     * @return The current angle of the module, as a `Rotation2d`.
     */
    private Rotation2d getAngle() {
        return new Rotation2d(steerMotor.getSelectedSensorPosition() * STEER_TICKS_TO_RADIANS);
    }

    private void setDriveP(EntryNotification change) {
        drivePidController.setP(change.value.getDouble());
    }

    private void setDriveI(EntryNotification change) {
        drivePidController.setI(change.value.getDouble());
    }

    private void setDriveD(EntryNotification change) {
        drivePidController.setD(change.value.getDouble());
    }

    private void setDriveFF(EntryNotification change) {
        drivePidController.setFF(change.value.getDouble());
    }

    private void setSteerP(EntryNotification change) {
        steerMotor.config_kP(0, change.value.getDouble());
    }

    private void setSteerI(EntryNotification change) {
        steerMotor.config_kI(0, change.value.getDouble());
    }

    private void setSteerD(EntryNotification change) {
        steerMotor.config_kD(0, change.value.getDouble());
    }

    private void setSteerFF(EntryNotification change) {
        steerMotor.config_kF(0, change.value.getDouble());
    }

    private void setDriveReference(EntryNotification change) {
        drivePidController.setReference(change.value.getDouble(), ControlType.kVelocity);
    }
}
