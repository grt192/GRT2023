package frc.robot.subsystems.swerve;

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

import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveSubsystem extends BaseSwerveSubsystem {
    private final SwerveModule.TopLeft topLeftModule;
    private final SwerveModule.TopRight topRightModule;
    private final SwerveModule.BottomLeft bottomLeftModule;
    private final SwerveModule.BottomRight bottomRightModule;

    private final SwerveDrivePoseEstimator poseEstimator;

    public static final double MAX_VEL = Units.feetToMeters(16.10); // Max robot tangential velocity, in m/s
    public static final double MAX_ACCEL = 3; // Max robot tangential acceleration, in m/s^2
    public static final double MAX_OMEGA = MAX_VEL / tlPos.getNorm(); // Max robot angular velocity, in rads/s (omega =
                                                                      // v / r)


    

    public SwerveSubsystem() {
        super(MAX_VEL, MAX_ACCEL, MAX_OMEGA, new SwerveDriveKinematics(
            tlPos, trPos, blPos, brPos));
        
        // Initialize swerve modules
        topLeftModule = new SwerveModule.TopLeft(tlDrive, tlSteer, tlOffsetRads);
        topRightModule = new SwerveModule.TopRight(trDrive, trSteer, trOffsetRads);
        bottomLeftModule = new SwerveModule.BottomLeft(blDrive, blSteer, blOffsetRads);
        bottomRightModule = new SwerveModule.BottomRight(brDrive, brSteer, brOffsetRads);


        // Initialize NaxX and pose estimator

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getGyroHeading(),
                getModuleStates(),
                new Pose2d(),
                // State measurement standard deviations: [X, Y, theta]
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
                // Vision measurement standard deviations: [X, Y, theta]
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));
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


}
