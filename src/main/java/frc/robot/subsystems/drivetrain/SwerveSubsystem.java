package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.vision.PhotonWrapper;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveSubsystem extends BaseSwerveSubsystem {
    public static final double MAX_VEL = 5.09346342086792 /* Units.feetToMeters(16.10) */; // Max robot tangential velocity, in m/s
    public static final double MAX_ACCEL = 3; // Max robot tangential acceleration, in m/s^2; TODO: measure this
    public static final double MAX_OMEGA = MAX_VEL / tlPos.getNorm(); // Max robot angular velocity, in rads/s (omega = v / r)
    public static final double MAX_ALPHA = 7.97564656352; // Max robot angular acceleration, in rads/s^2

    public SwerveSubsystem(PhotonWrapper photonWrapper) {
        super(
            new SwerveModule.TopLeft(tlDrive, tlSteer, tlOffsetRads),
            new SwerveModule.TopRight(trDrive, trSteer, trOffsetRads),
            new SwerveModule.BottomLeft(blDrive, blSteer, blOffsetRads),
            new SwerveModule.BottomRight(brDrive, brSteer, brOffsetRads),
            MAX_VEL, MAX_ACCEL, MAX_OMEGA, MAX_ALPHA,
            new SwerveDriveKinematics(tlPos, trPos, blPos, brPos),
            photonWrapper
        );
    }
}
