package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.SwerveConstants.*;

<<<<<<< HEAD:src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java
<<<<<<< HEAD
public class SwerveSubsystem extends BaseSwerveSubsystem {
=======
public class SwerveSubsystem extends SubsystemBase {
=======
public class SwerveSubsystem extends DriveTrain {
>>>>>>> 743ea1f (consolidated DT to DriveTrain class):src/main/java/frc/robot/subsystems/drivetrain/SwerveSubsystem.java

    private final SwerveModule.TopLeft topLeftModule;
    private final SwerveModule.TopRight topRightModule;
    private final SwerveModule.BottomLeft bottomLeftModule;
    private final SwerveModule.BottomRight bottomRightModule;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final AHRS ahrs;

    private final SwerveDriveKinematics kinematics;

>>>>>>> e11f83f (added second balancer constructor to allow for DT flexibility)
    public static final double MAX_VEL = Units.feetToMeters(16.10); // Max robot tangential velocity, in m/s
    public static final double MAX_ACCEL = 3; // Max robot tangential acceleration, in m/s^2
    public static final double MAX_OMEGA = MAX_VEL / tlPos.getNorm(); // Max robot angular velocity, in rads/s (omega = v / r)

    public SwerveSubsystem() {
<<<<<<< HEAD:src/main/java/frc/robot/subsystems/swerve/SwerveSubsystem.java
        super(
            new SwerveModule.TopLeft(tlDrive, tlSteer, tlOffsetRads),
            new SwerveModule.TopRight(trDrive, trSteer, trOffsetRads),
            new SwerveModule.BottomLeft(blDrive, blSteer, blOffsetRads),
            new SwerveModule.BottomRight(brDrive, brSteer, brOffsetRads),
            MAX_VEL, MAX_ACCEL, MAX_OMEGA, 
            new SwerveDriveKinematics(tlPos, trPos, blPos, brPos)
=======
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
    public void setDTDrivePowers() {
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
>>>>>>> 743ea1f (consolidated DT to DriveTrain class):src/main/java/frc/robot/subsystems/drivetrain/SwerveSubsystem.java
        );
    }
}
