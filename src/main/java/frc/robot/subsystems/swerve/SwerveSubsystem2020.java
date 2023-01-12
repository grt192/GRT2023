package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.SwerveConstants2020.*;

public class SwerveSubsystem2020 extends BaseSwerveSubsystem {


    public static final double MAX_VEL = 1.0; // Max robot tangential velocity, in m/s
    public static final double MAX_ACCEL = 3; // Max robot tangential acceleration, in m/s^2
    public static final double MAX_OMEGA = Math.toRadians(60); // Max robot angular velocity, in rads/s

    


    public SwerveSubsystem2020() {
        super(
            new SwerveModule2020(tlDrive, tlSteer, tlOffsetRads),
            new SwerveModule2020(trDrive, trSteer, trOffsetRads),
            new SwerveModule2020(blDrive, blSteer, blOffsetRads),
            new SwerveModule2020(brDrive, brSteer, brOffsetRads),
            MAX_VEL, MAX_ACCEL, MAX_OMEGA,new SwerveDriveKinematics(
            tlPos, trPos, blPos, brPos));
    }
    


}
