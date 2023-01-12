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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BaseSwerveSubsystem extends SubsystemBase {
    
    private final BaseSwerveModule topLeftModule;
    private final BaseSwerveModule topRightModule;
    private final BaseSwerveModule bottomLeftModule;
    private final BaseSwerveModule bottomRightModule;

    private final AHRS ahrs;
    private double angleoffset = 0;

    protected final SwerveDriveKinematics kinematics;

    public final double MAX_VEL; // Max robot tangential velocity, in m/s
    public final double MAX_ACCEL; // Max robot tangential acceleration, in m/s^2
    public final double MAX_OMEGA; // Max robot angular velocity, in rads/s

    protected final Timer lockTimer;
    protected static final double LOCK_TIMEOUT_SECONDS = 1.0; // The elapsed idle time to wait before locking
    protected static final boolean LOCKING_ENABLE = true;
    
    private final SwerveDrivePoseEstimator poseEstimator;

    // The driver or auton commanded `SwerveModuleState` setpoints for each module;
    // states are given in a tuple of [top left, top right, bottom left, bottom
    // right].
    protected SwerveModuleState[] states = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    public BaseSwerveSubsystem(BaseSwerveModule topLeftModule, BaseSwerveModule topRightModule, BaseSwerveModule bottomLeftModule, BaseSwerveModule bottomRightModule, double maxVel, double maxAccel, double maxOmega, SwerveDriveKinematics kinematics) {
        ahrs = new AHRS(SPI.Port.kMXP);
        MAX_VEL = maxVel;
        MAX_ACCEL = maxAccel;
        MAX_OMEGA = maxOmega;
        this.kinematics = kinematics;

        this.topLeftModule = topLeftModule;
        this.topRightModule = topRightModule;
        this.bottomLeftModule = bottomLeftModule;
        this.bottomRightModule = bottomRightModule;

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getGyroHeading(),
                getModuleStates(),
                new Pose2d(),
                // State measurement standard deviations: [X, Y, theta]
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
                // Vision measurement standard deviations: [X, Y, theta]
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));

        
        lockTimer = new Timer();
    }

    @Override
    public void periodic() {
        // Update pose estimator from swerve module states
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.update(
                gyroAngle,
                getModuleStates());

        // If all commanded velocities are 0, the system is idle (drivers / commands are
        // not supplying input).
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

        // Lock the swerve modules if the lock timeout has elapsed, or set them to their
        // setpoints if
        // drivers are supplying non-idle input.
        if (LOCKING_ENABLE && lockTimer.hasElapsed(LOCK_TIMEOUT_SECONDS)) {
            topLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
            topRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
        } else {
            // Desaturate speeds to ensure all velocities are under MAX_VEL after
            // kinematics.
            SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VEL);

            topLeftModule.setDesiredState(states[0]);
            topRightModule.setDesiredState(states[1]);
            bottomLeftModule.setDesiredState(states[2]);
            bottomRightModule.setDesiredState(states[3]);
        }
    }


    /**
     * Sets the swerve module states of this subsystem from provided field-centric
     * swerve drive powers.
     * 
     * @param xPower       The power [-1.0, 1.0] in the x (forward) direction.
     * @param yPower       The power [-1.0, 1.0] in the y (left) direction.
     * @param angularPower The angular (rotational) power [-1.0, 1.0].
     */
    public void setSwerveDrivePowers(double xPower, double yPower, double angularPower) {
        // If drivers are sending no input, stop all modules but hold their current
        // angle.
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
                getGyroForField());

        // Calculate swerve module states from desired chassis speeds
        this.states = kinematics.toSwerveModuleStates(speeds);
    }

    /**
     * Gets the states of each module as a `SwerveModulePosition[]`.
     * 
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
     * Sets the swerve module states of this subsystem. Module states are assumed to
     * be passed
     * in a tuple of [top left, top right, bottom left, bottom right].
     * 
     * @param states The swerve module states to set.
     */
    public void setSwerveModuleStates(SwerveModuleState... states) {
        this.states = states;
    }

    /**
     * Gets the subsystems `SwerveDriveKinematics` instance.
     * 
     * @return The SwerveDriveKinematics representing this system's kinematics.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Gets the estimated current position of the robot.
     * 
     * @return The estimated position of the robot as a Pose2d.
     */
    public Pose2d getRobotPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset the robot's position to a given Pose2d.
     * 
     * @param position The position to reset the pose estimator to.
     */
    public void resetPosition(Pose2d position) {
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.resetPosition(
                gyroAngle,
                getModuleStates(),
                position);
    }

    /**
     * Zeros the robot's position.
     * This method zeros both the robot's translation *and* rotation.
     */
    public void resetPosition() {
        resetPosition(new Pose2d());
    }

    public void resetFieldAngle() {
        angleoffset = ahrs.getAngle();
    }

    
    /**
     * Gets the gyro angle given by the NavX AHRS, inverted to be counterclockwise
     * positive.
     * 
     * @return The robot heading as a Rotation2d.
     */
    protected Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

    protected Rotation2d getGyroForField() {
        return Rotation2d.fromDegrees(-(ahrs.getAngle() - angleoffset));
    }

}
