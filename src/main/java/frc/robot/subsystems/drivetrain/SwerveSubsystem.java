package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.vision.PhotonWrapper;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveSubsystem extends BaseSwerveSubsystem {
    public static final double MAX_VEL = 5.09346342086792 /* Units.feetToMeters(16.10) */; // Max robot tangential velocity, in m/s
    public static final double MAX_ACCEL = 3; // Max robot tangential acceleration, in m/s^2; TODO: measure this
    public static final double MAX_OMEGA = MAX_VEL / TL_POS.getNorm(); // Max robot angular velocity, in rads/s (omega = v / r)
    public static final double MAX_ALPHA = 7.97564656352; // Max robot angular acceleration, in rads/s^2

    public SwerveSubsystem(PhotonWrapper photonWrapper, boolean VERBOSE_SHUFFLEBOARD) {
        super(
            new SwerveModule.TopLeft(TL_DRIVE, TL_STEER, TL_OFFSET_RADS, VERBOSE_SHUFFLEBOARD),
            new SwerveModule.TopRight(TR_DRIVE, TR_STEER, TR_OFFSET_RADS, VERBOSE_SHUFFLEBOARD),
            new SwerveModule.BottomLeft(BL_DRIVE, BL_STEER, BL_OFFSET_RADS, VERBOSE_SHUFFLEBOARD),
            new SwerveModule.BottomRight(BR_DRIVE, BR_STEER, BR_OFFSET_RADS, VERBOSE_SHUFFLEBOARD),
            MAX_VEL, MAX_ACCEL, MAX_OMEGA, MAX_ALPHA,
            new SwerveDriveKinematics(TL_POS, TR_POS, BL_POS, BR_POS),
            photonWrapper
        );
    }
}
