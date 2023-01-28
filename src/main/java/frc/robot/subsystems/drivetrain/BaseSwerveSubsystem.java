package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.OptionalDouble;

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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.PhotonWrapper;

/**
 * The superclass for the current `SwerveSubsystem` and `SwerveSubsystem2020` that contains all the
 * logic for managing module states, updating odometry, and taking driver input.
 */
public abstract class BaseSwerveSubsystem extends BaseDrivetrain {
    private final BaseSwerveModule topLeftModule;
    private final BaseSwerveModule topRightModule;
    private final BaseSwerveModule bottomLeftModule;
    private final BaseSwerveModule bottomRightModule;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveKinematics kinematics;

    private final PhotonWrapper photonWrapper;

    public final double MAX_VEL; // Max robot tangential velocity, in m/s
    public final double MAX_ACCEL; // Max robot tangential acceleration, in m/s^2
    public final double MAX_OMEGA; // Max robot angular velocity, in rads/s
    public final double MAX_ALPHA; // Max robot angular acceleration, in rads/s^2

    private final Timer lockTimer;
    private static final double LOCK_TIMEOUT_SECONDS = 1.0; // The elapsed idle time to wait before locking
    private static final boolean LOCKING_ENABLE = true;

    private Rotation2d angleOffset = new Rotation2d(0);

    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
    private final Field2d cfield = new Field2d();
    private final GenericEntry xEntry = shuffleboardTab.add("xpos", 0).getEntry();
    private final GenericEntry yEntry = shuffleboardTab.add("ypos", 0).getEntry();
    private final GenericEntry thetaEntry = shuffleboardTab.add("thetapos", 0).getEntry();

    private final GenericEntry timestampVisionEntry = shuffleboardTab.add("timestamp vision", 0).getEntry();
    private final GenericEntry xVisionEntry = shuffleboardTab.add("x vision pos", 0).getEntry();
    private final GenericEntry yVisionEntry = shuffleboardTab.add("y vision pos", 0).getEntry();

    // ----------- TESTING ------------
    private final GenericEntry xStatVisionEntry = shuffleboardTab.add("x vision stats", "").getEntry();
    private final GenericEntry yStatVisionEntry = shuffleboardTab.add("x vision stats", "").getEntry();

    private final GenericEntry estimationTimerEntry = shuffleboardTab.add("estimation timer", 0).getEntry();
    private Timer estimationTimer = new Timer();

    ArrayList<Double> xEntries = new ArrayList<Double>();
    ArrayList<Double> yEntries = new ArrayList<Double>();
    // --------------------------------

    // The driver or auton commanded `SwerveModuleState` setpoints for each module;
    // states are given in a tuple of [top left, top right, bottom left, bottom right].
    private SwerveModuleState[] states = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    public BaseSwerveSubsystem(
        BaseSwerveModule topLeftModule,
        BaseSwerveModule topRightModule,
        BaseSwerveModule bottomLeftModule,
        BaseSwerveModule bottomRightModule,
        double maxVel, double maxAccel, double maxOmega, double maxAlpha,
        SwerveDriveKinematics kinematics,
        PhotonWrapper photonWrapper
    ) {

        MAX_VEL = maxVel;
        MAX_ACCEL = maxAccel;
        MAX_OMEGA = maxOmega;
        MAX_ALPHA = maxAlpha;

        this.topLeftModule = topLeftModule;
        this.topRightModule = topRightModule;
        this.bottomLeftModule = bottomLeftModule;
        this.bottomRightModule = bottomRightModule;

        this.kinematics = kinematics;
        this.photonWrapper = photonWrapper;

        SmartDashboard.putData("Field", cfield);

        // Initialize pose estimator
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
        estimationTimer.start();
    }

    /**
     * Returns average, standard deviation, and range of a list of data.
     * @param data ArrayList of Doubles
     * @return array of average, standard deviation, and range
     */
    private double[] analyzeData(ArrayList<Double> data) {
        OptionalDouble optionalAverage = data.stream().mapToDouble(a -> a).average();

        final double average = (optionalAverage.isPresent() ? optionalAverage.getAsDouble() : 0);
        double stdDev = 0;
        double range = 0;

        // Calculate std dev if the double stream is present
        if (optionalAverage.isPresent()) {
            stdDev = Math.sqrt(data.stream().mapToDouble(a -> {
                return Math.pow((a - average), 2);
            }).average().getAsDouble());

            range = data.stream().mapToDouble(a -> a).max().getAsDouble() 
                - data.stream().mapToDouble(a -> a).min().getAsDouble();
        }

        return new double[] { average, stdDev, range };
    }

    @Override
    public void periodic() {
        // ----- TESTING CODE -------
        estimationTimerEntry.setValue(estimationTimer.get());

        if (estimationTimer.get() > 5.0) {
            double[] xStats = analyzeData(xEntries);
            double[] yStats = analyzeData(yEntries);

            xStatVisionEntry.setValue(String.format("%.3f %.3f %.3f", xStats[0], xStats[1], xStats[2]));
            yStatVisionEntry.setValue(String.format("%.3f %.3f %.3f", yStats[0], yStats[1], yStats[2]));
            
            xEntries.clear();
            yEntries.clear();
            estimationTimer.reset();
        }
        // --------------------------

        // Update pose estimator from swerve module states
        Rotation2d gyroAngle = getGyroHeading();
        
        Pose2d estimate = poseEstimator.update(
            gyroAngle,
            getModuleStates()
        );

        xEntry.setValue(Units.metersToInches(estimate.getX()));
        yEntry.setValue(Units.metersToInches(estimate.getY()));
        thetaEntry.setValue(estimate.getRotation().getDegrees());
        cfield.setRobotPose(estimate);

        // Add vision pose estimate to pose estimator
        photonWrapper.getRobotPose(estimate).forEach((visionPose) -> {

            // ---- TESTING ---            
            xEntries.add(visionPose.estimatedPose.getX());
            yEntries.add(visionPose.estimatedPose.getY());
            // ----------------

            xVisionEntry.setValue(Units.metersToInches(visionPose.estimatedPose.getX()));
            yVisionEntry.setValue(Units.metersToInches(visionPose.estimatedPose.getY()));
            timestampVisionEntry.setValue(visionPose.timestampSeconds);

            poseEstimator.addVisionMeasurement(
                visionPose.estimatedPose.toPose2d(),
                visionPose.timestampSeconds
            );
        });

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
        // setpoints if drivers are supplying non-idle input.
        if (LOCKING_ENABLE && lockTimer.hasElapsed(LOCK_TIMEOUT_SECONDS)) {
            topLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
            topRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
        } else {
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
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction.
     * @param yPower The power [-1.0, 1.0] in the y (left) direction.
     * @param angularPower The angular (rotational) power [-1.0, 1.0].
     * @param relative Whether to use relative powers instead of field-oriented control.
     */
    public void setDrivePowers(double xPower, double yPower, double angularPower, boolean relative) {
        // Scale [-1.0, 1.0] powers to desired velocity, turning field-relative powers
        // into robot relative chassis speeds.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * MAX_VEL,
            yPower * MAX_VEL,
            angularPower * MAX_OMEGA,
            relative ? new Rotation2d() : getFieldHeading()
        );

        // Calculate swerve module states from desired chassis speeds, desaturating
        // them to ensure all velocities are under MAX_VEL after kinematics.
        this.states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            this.states, speeds, 
            MAX_VEL, MAX_VEL, MAX_OMEGA
        );
    }

    /**
     * Sets the swerve module states of this subsystem from provided relative drive powers.
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction, relative to the robot.
     */
    @Override
    public void setDrivePowers(double xPower) {
        setDrivePowers(xPower, 0.0, 0.0, true);
    }

    /**
     * Sets the swerve module states of this subsystem. Module states are assumed to
     * be passed in a tuple of [top left, top right, bottom left, bottom right].
     * 
     * @param states The swerve module states to set.
     */
    public void setSwerveModuleStates(SwerveModuleState... states) {
        this.states = states;
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
     * Gets the subsystems `SwerveDriveKinematics` instance.
     * @return The SwerveDriveKinematics representing this system's kinematics.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
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
     * @param currentPose The position to reset the pose estimator to.
     */
    public void resetPose(Pose2d currentPose) {
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.resetPosition(
            gyroAngle,
            getModuleStates(),
            currentPose
        );

        angleOffset = gyroAngle.minus(currentPose.getRotation());
    }

    /**
     * Zeros the robot's position.
     * This method zeros both the robot's translation *and* rotation.
     */
    public void resetPose() {
        resetPose(new Pose2d());
    }

    /**
     * Zeros *only the angle* of the robot's field-relative control system.
     * This also resets localization to match the rotated field.
     */
    public void resetFieldAngle(Rotation2d currentRotation) {
        Pose2d currPos = getRobotPosition();

        // Reset localization angle but keep current (x, y) to preserve the origin.
        resetPose(new Pose2d(
            currPos.getTranslation(),
            currentRotation
        ));
    }

    public void resetFieldAngle() {
        resetFieldAngle(new Rotation2d());
    }

    /**
     * Gets the gyro angle given by the NavX AHRS, inverted to be counterclockwise positive.
     * @return The robot's global heading as a Rotation2d.
     */
    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

    /**
     * Gets the angle of the robot relative to the field-relative control system.
     * This is equivalent to the robot's global angle with an offset applied.
     * 
     * @return The robot's field-centric heading as a Rotation2d.
     */
    private Rotation2d getFieldHeading() {
        // Primarily use AHRS reading, falling back on the pose estimator if the AHRS disconnects.
        return ahrs.isConnected()
            ? getGyroHeading().minus(angleOffset)
            : getRobotPosition().getRotation();
    }
}
