package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.SwerveConstants.*;


public class SwerveSubsystem extends BaseSwerveSubsystem {
    

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
