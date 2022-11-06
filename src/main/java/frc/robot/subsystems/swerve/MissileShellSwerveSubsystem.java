package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.motorcontrol.MotorUtil;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.shuffleboard.GRTShuffleboardTab;

/**
 * A shell swerve subsystem to run a single swerve module on the missile.
 */
public class MissileShellSwerveSubsystem extends SubsystemBase {
    private final CANSparkMax driveMotor;

    private final CANSparkMax steerMotor;
    private final RelativeEncoder steerEncoder;
    private final SparkMaxAnalogSensor steerAbsoluteEncoder;
    private final SparkMaxPIDController steerPidController;

    private final SwerveDriveKinematics kinematics;

    public static final double MAX_VEL = 1.0; // Max robot tangential velocity, in percent output
    private static final double offsetRads = 0;

    private static final double STEER_ROTATIONS_TO_RADIANS = 1.0 / 39.0 * 2 * Math.PI; // 39:1 gear ratio, 1 rotation = 2pi
    private static final double STEER_VOLTS_TO_RADIANS = 2 * Math.PI / 3.3; // MA3 analog output: 3.3V -> 2pi

    private static final double steerP = 0.4;
    private static final double steerI = 0;
    private static final double steerD = 0;
    private static final double steerFF = 0;

    private SwerveModuleState[] states = {
        new SwerveModuleState()
    };

    private final GRTShuffleboardTab shuffleboardTab = new GRTShuffleboardTab("Swerve");
    private final GRTNetworkTableEntry positionEntry, setpointEntry;

    public MissileShellSwerveSubsystem() {
        driveMotor = MotorUtil.createSparkMax(2);

        steerMotor = MotorUtil.createSparkMax(1);
        steerMotor.setIdleMode(IdleMode.kBrake);

        steerAbsoluteEncoder = steerMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        steerAbsoluteEncoder.setPositionConversionFactor(STEER_VOLTS_TO_RADIANS);

        steerEncoder = steerMotor.getEncoder();
        steerEncoder.setPositionConversionFactor(STEER_ROTATIONS_TO_RADIANS);
        steerEncoder.setPosition(steerAbsoluteEncoder.getPosition()); // Set initial position to absolute value

        steerPidController = steerMotor.getPIDController();
        //steerPidController.setFeedbackDevice(steerEncoder);
        steerPidController.setP(steerP);
        steerPidController.setI(steerI);
        steerPidController.setD(steerD);
        steerPidController.setFF(steerFF);

        // One module at the center of the robot
        kinematics = new SwerveDriveKinematics(
            new Translation2d(),
            new Translation2d()
        );

        positionEntry = shuffleboardTab.addEntry("Position", 0).at(0, 0);
        setpointEntry = shuffleboardTab.addEntry("Setpoint", 0).at(1, 0);

        shuffleboardTab
            .addListener("Steer p", steerP, this::setSteerP, 0, 1)
            .addListener("Steer i", steerI, this::setSteerI, 0, 2)
            .addListener("Steer d", steerD, this::setSteerD, 0, 3)
            .addListener("Steer ff", steerFF, this::setSteerFF, 0, 4);
    }

    @Override
    public void periodic() {
        positionEntry.setValue(Math.toDegrees(steerEncoder.getPosition()));
        setpointEntry.setValue(states[0].angle.getDegrees());

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VEL);
        setDesiredState(states[0]);
    }

    /**
     * Sets the desired state of the module.
     * @param state The desired state of the module as a `SwerveModuleState`.
     */
    public void setDesiredState(SwerveModuleState state) {
        var optimized = SwerveModule.optimizeModuleState(state, getAngle());
        //driveMotor.set(ControlMode.Velocity, optimized.getFirst() / (DRIVE_TICKS_TO_METERS * 10.0));
        driveMotor.set(optimized.getFirst());
        steerPidController.setReference(optimized.getSecond() - offsetRads, ControlType.kPosition);
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
     * Set the field-centric swerve drive powers of the subsystem.
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction.
     * @param yPower The power [-1.0, 1.0] in the y (left) direction.
     * @param angularPower The angular (rotational) power [-1.0, 1.0].
     */
    public void setSwerveDrivePowers(double xPower, double yPower, double angularPower) {
        // Scale [-1.0, 1.0] powers to desired velocity, turning field-relative powers
        // into robot relative chassis speeds.
        // For the missile's single-module setup, we assume that angular power is always 0 and
        // the "robot" is always facing forward.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * MAX_VEL, 
            yPower * MAX_VEL, 
            0,
            new Rotation2d()
        );

        // Calculate swerve module states from desired chassis speeds.
        this.states = kinematics.toSwerveModuleStates(speeds);
    }

    private void setSteerP(EntryNotification change) {
        steerPidController.setP(change.value.getDouble());
    }

    private void setSteerI(EntryNotification change) {
        steerPidController.setI(change.value.getDouble());
    }

    private void setSteerD(EntryNotification change) {
        steerPidController.setD(change.value.getDouble());
    }

    private void setSteerFF(EntryNotification change) {
        steerPidController.setFF(change.value.getDouble());
    }
}
