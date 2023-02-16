package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.vision.PhotonWrapper;

import static frc.robot.Constants.SwerveConstants2020.*;

public class SwerveSubsystem2020 extends BaseSwerveSubsystem {
    public static final double MAX_VEL = 1.0; // Max robot tangential velocity, in m/s
    public static final double MAX_ACCEL = 3; // Max robot tangential acceleration, in m/s^2
    public static final double MAX_OMEGA = Math.toRadians(60); // Max robot angular velocity, in rads/s
    public static final double MAX_ALPHA = 3; // Max robot angular acceleration, in rads/s^2

    public SwerveSubsystem2020(PhotonWrapper photonWrapper) {
        super(
            new SwerveModule2020(TL_DRIVE, TL_STEER, TL_OFFSET_RADS),
            new SwerveModule2020(TR_DRIVE, TR_STEER, TR_OFFSET_RADS),
            new SwerveModule2020(BL_DRIVE, BL_STEER, BL_OFFSET_RADS),
            new SwerveModule2020(BR_DRIVE, BR_STEER, BR_OFFSET_RADS),
            MAX_VEL, MAX_ACCEL, MAX_OMEGA, MAX_ALPHA,
            new SwerveDriveKinematics(TL_POS, TR_POS, BL_POS, BR_POS),
            photonWrapper
        );
    }
}
