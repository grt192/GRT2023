package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.SwerveConstants.*;

<<<<<<< HEAD
public class SwerveSubsystem extends BaseSwerveSubsystem {
=======
public class SwerveSubsystem extends SubsystemBase {

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
        super(
            new SwerveModule.TopLeft(tlDrive, tlSteer, tlOffsetRads),
            new SwerveModule.TopRight(trDrive, trSteer, trOffsetRads),
            new SwerveModule.BottomLeft(blDrive, blSteer, blOffsetRads),
            new SwerveModule.BottomRight(brDrive, brSteer, brOffsetRads),
            MAX_VEL, MAX_ACCEL, MAX_OMEGA, 
            new SwerveDriveKinematics(tlPos, trPos, blPos, brPos)
        );
    }
}
