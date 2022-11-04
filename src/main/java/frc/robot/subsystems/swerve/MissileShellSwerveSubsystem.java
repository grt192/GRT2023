package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    private final SparkMaxAnalogSensor steerEncoder;
    private final SparkMaxPIDController steerPidController;

    private final SwerveDriveKinematics kinematics;

    public static final double MAX_VEL = 1.0; // Max robot tangential velocity, in percent output
    private static final double offsetRads = 0;

    private SwerveModuleState[] states = {
        new SwerveModuleState()
    };

    private final GRTShuffleboardTab shuffleboardTab = new GRTShuffleboardTab("Swerve");

    public MissileShellSwerveSubsystem() {
        driveMotor = MotorUtil.createSparkMax(2);

        steerMotor = MotorUtil.createSparkMax(1);
        steerMotor.setIdleMode(IdleMode.kCoast); // TODO: investigate strange braking behavior

        steerEncoder = steerMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        steerEncoder.setPositionConversionFactor(2 * Math.PI / 3.3); // 3.3V -> 2pi

        steerPidController = steerMotor.getPIDController();
        steerPidController.setFeedbackDevice(steerEncoder);

        // One module at the center of the robot
        kinematics = new SwerveDriveKinematics(
            new Translation2d(),
            new Translation2d()
        );
    }

    @Override
    public void periodic() {
        steerMotor.set(0.4);
        driveMotor.set(0.4);
        System.out.println(Math.toDegrees(steerEncoder.getPosition()));

        /*
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VEL);
        setDesiredState(states[0]);
        */
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
}
