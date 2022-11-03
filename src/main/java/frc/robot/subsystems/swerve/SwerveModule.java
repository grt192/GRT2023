package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.networktables.EntryNotification;

import frc.robot.motorcontrol.MotorUtil;
// import frc.robot.shuffleboard.GRTNetworkTableEntry;
// import frc.robot.shuffleboard.GRTShuffleboardTab;

/**
 * A swerve module with a NEO drive motor and a BAG steer motor, for running swerve modules on 
 * the 2020 robot.
 */
public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePidController;

    private final WPI_TalonSRX steerMotor;

    private final double offsetRads;

    private static final double DRIVE_ROTATIONS_TO_METERS = 60.0;
    private static final double STEER_TICKS_TO_RADIANS = 2 * Math.PI / 1024.0;

    private static final double driveP = 0;
    private static final double driveI = 0;
    private static final double driveD = 0;
    private static final double driveFF = 0;

    private static final double steerP = 5.5;
    private static final double steerI = 0;
    private static final double steerD = 0;
    private static final double steerFF = 0;

    // private final GRTShuffleboardTab shuffleboardTab;
    // private final GRTNetworkTableEntry veloEntry, angleEntry;

    public SwerveModule(int drivePort, int steerPort, double offsetRads) {
        
        driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setVelocityConversionFactor(DRIVE_ROTATIONS_TO_METERS / 60.0); // RPM -> m/s

        drivePidController = driveMotor.getPIDController();
        drivePidController.setP(driveP);
        drivePidController.setI(driveI);
        drivePidController.setD(driveD);
        drivePidController.setFF(driveFF);

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

        /*
        shuffleboardTab = new GRTShuffleboardTab("Swerve " + drivePort);

        veloEntry = shuffleboardTab.addEntry("Vel", driveEncoder.getVelocity()).at(0, 0);
        angleEntry = shuffleboardTab.addEntry("Angle", getAngle().getRadians()).at(0, 1);

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
        */
    }

    public SwerveModule(int drivePort, int steerPort) {
        this(drivePort, steerPort, 0.0);
    }

    /**
     * Testing function to get the current velocity of the drive motor.
     * @return The current velocity of the drive motor, in RPM.
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Gets the current state of the module as a `SwerveModuleState`.
     * @return The state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.get(), // NOTE: this is only while the wheel velocity is in percent output instead
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

        // angleEntry.setValue(getAngle().getRadians());
        // veloEntry.setValue(driveEncoder.getVelocity());
    }

    /**
     * Returns the current angle of the module. This differs from the raw encoder reading 
     * because this applies `offsetRads` to zero the module at a desired angle.
     * 
     * @return The current angle of the module, as a `Rotation2d`.
     */
    private Rotation2d getAngle() {
        return new Rotation2d(
            steerMotor.getSelectedSensorPosition() * STEER_TICKS_TO_RADIANS + offsetRads
        );
    }
    
    /*
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
    */
    
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