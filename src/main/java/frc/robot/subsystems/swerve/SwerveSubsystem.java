package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule.TopLeft topLeftModule;
    private final SwerveModule.TopRight topRightModule;
    private final SwerveModule.BottomLeft bottomLeftModule;
    private final SwerveModule.BottomRight bottomRightModule;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final AHRS ahrs;

    private final SwerveDriveKinematics kinematics;

    public static final double MAX_VEL = Units.feetToMeters(16.10); // Max robot tangential velocity, in m/s
    public static final double MAX_ACCEL = 3; // Max robot tangential acceleration, in m/s^2
    public static final double MAX_OMEGA = MAX_VEL / tlPos.getNorm(); // Max robot angular velocity, in rads/s (omega = v / r)

    private final Timer lockTimer;
    private static final double LOCK_TIMEOUT_SECONDS = 1.0; // The elapsed idle time to wait before locking
    private static final boolean LOCKING_ENABLE = true;

    // The driver or auton commanded `SwerveModuleState` setpoints for each module;
    // states are given in a tuple of [top left, top right, bottom left, bottom right].
    private SwerveModuleState[] states = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    public SwerveSubsystem() {
        // Initialize swerve modules
        topLeftModule = new SwerveModule.TopLeft(tlDrive, tlSteer, tlOffsetRads);
        topRightModule = new SwerveModule.TopRight(trDrive, trSteer, trOffsetRads);
        bottomLeftModule = new SwerveModule.BottomLeft(blDrive, blSteer, blOffsetRads);
        bottomRightModule = new SwerveModule.BottomRight(brDrive, brSteer, brOffsetRads);

        // Initialize system kinematics with top left, top right, bottom left, and bottom right swerve
        // module positions.
        kinematics = new SwerveDriveKinematics(
            tlPos, trPos, blPos, brPos
        );

        // Initialize NaxX and pose estimator
        ahrs = new AHRS(SPI.Port.kMXP);
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroHeading(),
            getModuleStates(),
            new Pose2d(), 
            // State measurement standard deviations: [X, Y, theta]
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), 
            // Vision measurement standard deviations: [X, Y, theta]
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        );

        lockTimer = new Timer();
    }

    /**
     * Sets the swerve module states of this subsystem from provided field-centric swerve drive powers.
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction.
     * @param yPower The power [-1.0, 1.0] in the y (left) direction.
     * @param angularPower The angular (rotational) power [-1.0, 1.0].
     */
    public void setSwerveDrivePowers(double xPower, double yPower, double angularPower) {
        // If drivers are sending no input, stop all modules but hold their current angle.
        if (xPower == 0.0 && yPower == 0.0 && angularPower == 0.0) {
            this.states[0] = new SwerveModuleState(0.0, this.states[0].angle);
            this.states[1] = new SwerveModuleState(0.0, this.states[1].angle);
            this.states[2] = new SwerveModuleState(0.0, this.states[2].angle);
            this.states[3] = new SwerveModuleState(0.0, this.states[3].angle);
            return;
        }

        // Scale [-1.0, 1.0] powers to desired velocity, turning field-relative powers
        // into robot relative chassis speeds.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * MAX_VEL, 
            yPower * MAX_VEL, 
            angularPower * MAX_OMEGA,
            getGyroHeading()
        );

        // Calculate swerve module states from desired chassis speeds
        this.states = kinematics.toSwerveModuleStates(speeds);
    }

    /**
     * Sets the swerve module states of this subsystem. Module states are assumed to be passed
     * in a tuple of [top left, top right, bottom left, bottom right].
     * @param states The swerve module states to set.
     */
    public void setSwerveModuleStates(SwerveModuleState... states) {
        this.states = states;
    }

    /**
     * Gets the subsystems `SwerveDriveKinematics` instance.
     * @return The SwerveDriveKinematics representing this system's kinematics.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void periodic() {
        // Update pose estimator from swerve module states
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.update(
            gyroAngle,
            getModuleStates()
        );

        // If all commanded velocities are 0, the system is idle (drivers / commands are not supplying input).
        boolean isIdle = states[0].speedMetersPerSecond == 0.0
            && states[1].speedMetersPerSecond == 0.0
            && states[2].speedMetersPerSecond == 0.0
            && states[3].speedMetersPerSecond == 0.0;

        // Start lock timer when idle
        if (isIdle) {
            lockTimer.start();
        } else {
            lockTimer.stop();
            lockTimer.reset();
        }

        // Lock the swerve modules if the lock timeout has elapsed, or set them to their setpoints if
        // drivers are supplying non-idle input.
        if (LOCKING_ENABLE && lockTimer.hasElapsed(LOCK_TIMEOUT_SECONDS)) {
            topLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
            topRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
        } else {
            // Desaturate speeds to ensure all velocities are under MAX_VEL after kinematics.
            SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VEL);

            topLeftModule.setDesiredState(states[0]);
            topRightModule.setDesiredState(states[1]);
            bottomLeftModule.setDesiredState(states[2]);
            bottomRightModule.setDesiredState(states[3]);
        }
    }

    /**
     * Gets the states of each module as a `SwerveModulePosition[]`.
     * @return The states of each module.
     */
    private SwerveModulePosition[] getModuleStates() {
        return new SwerveModulePosition[] {
            topLeftModule.getState(),
            topRightModule.getState(),
            bottomLeftModule.getState(),
            bottomRightModule.getState()
        };
    }

    /**
     * Gets the estimated current position of the robot.
     * @return The estimated position of the robot as a Pose2d.
     */
    public Pose2d getRobotPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset the robot's position to a given Pose2d.
     * @param position The position to reset the pose estimator to.
     */
    public void resetPosition(Pose2d position) {
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.resetPosition(
            gyroAngle,
            getModuleStates(),
            position
        );
    }

    /**
     * Zeros the robot's position.
     * This method zeros both the robot's translation *and* rotation.
     */
    public void resetPosition() {
        resetPosition(new Pose2d());
    }

    /**
     * Gets the gyro angle given by the NavX AHRS, inverted to be counterclockwise positive.
     * @return The robot heading as a Rotation2d.
     */
    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }
}
