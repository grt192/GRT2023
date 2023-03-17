package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.util.FieldUtil;
import frc.robot.util.ShuffleboardUtil;
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
    private final LEDSubsystem ledSubsystem;

    public final double MAX_VEL; // Max robot tangential velocity, in m/s
    public final double MAX_ACCEL; // Max robot tangential acceleration, in m/s^2
    public final double MAX_OMEGA; // Max robot angular velocity, in rads/s
    public final double MAX_ALPHA; // Max robot angular acceleration, in rads/s^2

    // private final ProfiledPIDController thetaController;
    private final PIDController thetaController;

    private final Timer lockTimer;
    private static final double LOCK_TIMEOUT_SECONDS = 1.0; // The elapsed idle time to wait before locking
    private static final boolean LOCKING_ENABLE = true;
    private boolean chargingStationLocked = false;

    private Rotation2d driverHeadingOffset = new Rotation2d(0);

    private final ShuffleboardTab shuffleboardTab;
    private final GenericEntry xEntry, yEntry, thetaEntry;
    private final GenericEntry swerveRelativeEntry, chargingStationLockedEntry, relativeEncoderEntry, visionEnableEntry;

    private final Field2d fieldWidget = new Field2d();

    private static final boolean SHUFFLEBOARD_ENABLE = true;
    private volatile boolean VISION_ENABLE = true;

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
        PhotonWrapper photonWrapper,
        LEDSubsystem ledSubsystem
    ) {
        MAX_VEL = maxVel;
        MAX_ACCEL = maxAccel;
        MAX_OMEGA = maxOmega;
        MAX_ALPHA = maxAlpha;

        // Initialize heading-lock PID controller
        /*
        thetaController = new ProfiledPIDController(
            1.7, 0, 0, 
            new TrapezoidProfile.Constraints(MAX_OMEGA, MAX_ALPHA)
        );
        */
        thetaController = new PIDController(4.6, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.topLeftModule = topLeftModule;
        this.topRightModule = topRightModule;
        this.bottomLeftModule = bottomLeftModule;
        this.bottomRightModule = bottomRightModule;

        this.kinematics = kinematics;
        this.photonWrapper = photonWrapper;
        this.ledSubsystem = ledSubsystem;

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

        shuffleboardTab = Shuffleboard.getTab("Driver");
        shuffleboardTab.add("Field", fieldWidget)
            .withPosition(8, 1)
            .withSize(3, 2);
        xEntry = shuffleboardTab.add("x pos (in)", 0).withPosition(0, 5).getEntry();
        yEntry = shuffleboardTab.add("y pos (in)", 0).withPosition(1, 5).getEntry();
        thetaEntry = shuffleboardTab.add("theta pos (deg)", 0).withPosition(2, 5).getEntry();

        swerveRelativeEntry = shuffleboardTab.add("Swerve relative", false)
            .withPosition(3, 5)
            .getEntry();
        chargingStationLockedEntry = shuffleboardTab.add("Charging station locking", chargingStationLocked)
            .withPosition(4, 5)
            .getEntry();
        relativeEncoderEntry = shuffleboardTab.add("Relative encoder feedback", false)
            .withPosition(5, 5)
            .getEntry();

        visionEnableEntry = Shuffleboard.getTab("PhotonVision").add("Vision enable", VISION_ENABLE)
            .withPosition(3, 0)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
        ShuffleboardUtil.addBooleanListener(visionEnableEntry, (value) -> VISION_ENABLE = value);

        GenericEntry relativeEncoderToggleEntry = shuffleboardTab.add("Relative encoder feedback (set)", false)
            .withPosition(6, 5)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
        ShuffleboardUtil.addBooleanListener(relativeEncoderToggleEntry, (value) -> setSteerRelativeEncoderFeedback(value));

        lockTimer = new Timer();
    }

    @Override
    public void periodic() {
        // Update pose estimator from swerve module states
        Rotation2d gyroAngle = getGyroHeading();
        Pose2d estimate = poseEstimator.update(
            gyroAngle,
            getModuleStates()
        );

        // Update Shuffleboard
        if (SHUFFLEBOARD_ENABLE) {
            xEntry.setValue(Units.metersToInches(estimate.getX()));
            yEntry.setValue(Units.metersToInches(estimate.getY()));
            thetaEntry.setValue(estimate.getRotation().getDegrees());
            fieldWidget.setRobotPose(estimate);
        }

        // Add vision pose estimate to pose estimator
        if (VISION_ENABLE) photonWrapper.getRobotPoses(estimate).forEach((visionPose) -> {
            if (!FieldUtil.poseInField(visionPose.estimatedPose.toPose2d())) return;

            if (ledSubsystem != null) ledSubsystem.displayTagDetected();
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

        // Lock the swerve module if the lock timeout has elapsed, or set them to their 
        // setpoints if drivers are supplying non-idle input.
        if (lockTimer.hasElapsed(LOCK_TIMEOUT_SECONDS)) {
            applyLock();
        } else {
            topLeftModule.setDesiredState(states[0]);
            topRightModule.setDesiredState(states[1]);
            bottomLeftModule.setDesiredState(states[2]);
            bottomRightModule.setDesiredState(states[3]);
        }
    }

    /**
     * Locks the swerve. This sets the wheels parallel to the charging station if the charging
     * station lock mode is enabled, or in an X otherwise.
     */
    public void applyLock() {
        if (chargingStationLocked) {
            // Lock modules parallel to the charging station, accounting for the orientation of the robot.
            Rotation2d lockAngle = Rotation2d.fromDegrees(90).minus(getDriverHeading());

            topLeftModule.setDesiredState(new SwerveModuleState(0.0, lockAngle));
            topRightModule.setDesiredState(new SwerveModuleState(0.0, lockAngle));
            bottomLeftModule.setDesiredState(new SwerveModuleState(0.0, lockAngle));
            bottomRightModule.setDesiredState(new SwerveModuleState(0.0, lockAngle));
        } else if (LOCKING_ENABLE) {
            topLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
            topRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
            bottomRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
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
            relative ? new Rotation2d() : getDriverHeading()
        );

        // Calculate swerve module states from desired chassis speeds, desaturating
        // them to ensure all velocities are under MAX_VEL after kinematics.
        this.states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            this.states, speeds, 
            MAX_VEL, MAX_VEL, MAX_OMEGA
        );

        swerveRelativeEntry.setBoolean(relative);
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
     * Sets the swerve module states of this subsystem from provided field-centric
     * swerve drive powers, locking the swerve to a given heading.
     * 
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction.
     * @param yPower The power [-1.0, 1.0] in the y (left) direction.
     * @param targetHeading The `Rotation2d` angle to lock the swerve to.
     * @param relative Whether to use relative powers instead of field-oriented control.
     */
    public void setDrivePowersWithHeadingLock(double xPower, double yPower, Rotation2d targetHeading, boolean relative) {
        Rotation2d currentRotation = getDriverHeading();
        double turnSpeed = thetaController.calculate(currentRotation.getRadians(), targetHeading.getRadians());
        double turnPower = MathUtil.clamp(turnSpeed / MAX_OMEGA, -1.0, 1.0);

        // System.out.println("curr: " + currentRotation + " eror: " + Units.radiansToDegrees(error) + " turnsped: " + turnSpeed);

        setDrivePowers(xPower, yPower, turnPower, relative);
    }

    /**
     * Sets the swerve module states of this subsystem. Module states are assumed to
     * be passed in a tuple of [top left, top right, bottom left, bottom right].
     * 
     * @param states The swerve module states to set.
     */
    public void setSwerveModuleStates(SwerveModuleState... states) {
        swerveRelativeEntry.setBoolean(false); // TODO: better way of setting this to false during auton
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
     * Sets whether the swerve should be locked parallel to the charging station.
     * @param locked Whether to lock the swerve parallel to the charging station.
     */
    public void setChargingStationLocked(boolean locked) {
        this.chargingStationLocked = locked;
        chargingStationLockedEntry.setBoolean(chargingStationLocked);
    }

    /**
     * Toggles whether the swerve should be locked parallel to the charging station.
     */
    public void toggleChargingStationLocked() {
        setChargingStationLocked(!chargingStationLocked);
    }

    /**
     * Sets whether the swerve modules should use integrated relative encoders (instead of
     * external absolute encoders) to steer.
     * 
     * @param useRelative Whether to use relative encoders as steer feedback.
     */
    public void setSteerRelativeEncoderFeedback(boolean useRelative) {
        topLeftModule.setSteerRelativeFeedback(useRelative);
        topRightModule.setSteerRelativeFeedback(useRelative);
        bottomLeftModule.setSteerRelativeFeedback(useRelative);
        bottomRightModule.setSteerRelativeFeedback(useRelative);

        relativeEncoderEntry.setBoolean(useRelative);
    }

    /**
     * Gets the subsystems `SwerveDriveKinematics` instance.
     * @return The SwerveDriveKinematics representing this system's kinematics.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Gets the subsystems heading-correction `PIDController` instance.
     * @return The PIDController for correcting robot heading.
     */
    public PIDController getThetaController() {
        return thetaController;
    }

    /**
     * Gets the estimated current position of the robot.
     * @return The estimated position of the robot as a Pose2d.
     */
    public Pose2d getRobotPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset the robot's position to a given Pose2d. This method does not reset the field heading.
     * @param currentPose The position to reset the pose estimator to.
     */
    public void resetPose(Pose2d currentPose) {
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.resetPosition(
            gyroAngle,
            getModuleStates(),
            currentPose
        );
    }

    /**
     * Zeros the robot's position. This method zeros both the robot's translation *and* rotation,
     * but does not reset the field heading.
     */
    public void resetPose() {
        resetPose(new Pose2d());
    }

    /**
     * Zeros *only the angle* of the robot's field-relative control system.
     * This method has no effect on odometry's origin.
     * @param currentRotation The rotation to reset the driver angle to.
     */
    public void resetDriverHeading(Rotation2d currentRotation) {
        driverHeadingOffset = getGyroHeading().minus(currentRotation);
    }

    /**
     * Zeros *only the angle* of the robot's field-relative control system.
     * This method has no effect on odometry's origin.
     */
    public void resetDriverHeading() {
        resetDriverHeading(new Rotation2d());
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
    public Rotation2d getDriverHeading() {
        // Primarily use AHRS reading, falling back on the pose estimator if the AHRS disconnects.
        Rotation2d robotHeading = ahrs.isConnected()
            ? getGyroHeading()
            : getRobotPosition().getRotation();

        return robotHeading.minus(driverHeadingOffset);
    }

    /**
     * Sets whether vision data is enabled.
     * @param visionEnable Whether to enable vision data for localization.
     */
    public void setVisionEnabled(boolean visionEnable) {
        this.VISION_ENABLE = visionEnable;
        visionEnableEntry.setBoolean(VISION_ENABLE);
    }
}
