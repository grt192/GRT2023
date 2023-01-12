package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.SwerveConstants2020.*;

public class SwerveSubsystem2020 extends BaseSwerveSubsystem {
    private final SwerveModule2020 topLeftModule;
    private final SwerveModule2020 topRightModule;
    private final SwerveModule2020 bottomLeftModule;
    private final SwerveModule2020 bottomRightModule;


    public static final double MAX_VEL = 1.0; // Max robot tangential velocity, in m/s
    public static final double MAX_ACCEL = 3; // Max robot tangential acceleration, in m/s^2
    public static final double MAX_OMEGA = Math.toRadians(60); // Max robot angular velocity, in rads/s

    


    public SwerveSubsystem2020() {
        super(MAX_VEL, MAX_ACCEL, MAX_OMEGA,new SwerveDriveKinematics(
            tlPos, trPos, blPos, brPos));
        
        // Initialize swerve modules
        topLeftModule = new SwerveModule2020(tlDrive, tlSteer, tlOffsetRads);
        topRightModule = new SwerveModule2020(trDrive, trSteer, trOffsetRads);
        bottomLeftModule = new SwerveModule2020(blDrive, blSteer, blOffsetRads);
        bottomRightModule = new SwerveModule2020(brDrive, brSteer, brOffsetRads);



    }
    

    @Override
    public void periodic() {
        // If all commanded velocities are 0, the system is idle (drivers are not
        // supplying input).
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


}
