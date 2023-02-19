package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.SwerveConstants;

/**
 * A shell swerve subsystem to run a single swerve module on the missile.
 */
public class MissileShellSwerveSubsystem extends BaseDrivetrain {
    private final SwerveModule module;
    private final SwerveDriveKinematics kinematics;

    public static final double MAX_VEL = SwerveSubsystem.MAX_VEL; // Max robot tangential velocity, in m/s
    private static final boolean OFFSET_TUNING_ENABLE = true; // Whether to use this subsystem for swerve module offset tuning.

    private SwerveModuleState[] states = {
        new SwerveModuleState()
    };

    public MissileShellSwerveSubsystem() {
        module = new SwerveModule.BottomRight(
            SwerveConstants.BR_DRIVE,
            SwerveConstants.BR_STEER,
            SwerveConstants.BR_OFFSET_RADS
        );

        // One module at the center of the robot
        kinematics = new SwerveDriveKinematics(
            new Translation2d(),
            new Translation2d()
        );
    }

    @Override
    public void periodic() {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VEL);
        module.setDesiredState(states[0]);

        // Print values for encoder tuning. To set a zero, currentAngle + offset = 0 -> offset = -currentAngle.
        // In case the offset is 180 degrees off, also print it plus pi.
        if (OFFSET_TUNING_ENABLE) {
            double currentAngleRads = module.getRawAngleRads();

            // The offset of the module assuming that it is aligned with the front of the robot.
            // For the robot offset, subtract the position offset automatically applied by the module subclasses.
            // currentAngle + offset + posOffset = 0 -> offset = -currentAngle - posOffset.
            double robotOffset = MathUtil.angleModulus(-currentAngleRads - module.getPositionOffsetRads());
            double flippedRobotOffset = MathUtil.angleModulus(-currentAngleRads - module.getPositionOffsetRads() + Math.PI);

            // The offset of the module assuming that it is aligned with its aligning pin.
            // For the pin offset, subtract pi/2 to account for the pin being 90 degrees off.
            // currentAngle + offset + pi/2 = 0 -> offset = -currentAngle - pi/2.
            double pinOffset = MathUtil.angleModulus(-currentAngleRads - Math.PI / 2);
            double flippedPinOffset = MathUtil.angleModulus(-currentAngleRads + Math.PI / 2);

            System.out.println(
                "-".repeat(20)
                + "\ncurrent angle: " + currentAngleRads
                + "\nrobot offset: " + robotOffset + ", robot offset (flipped): " + flippedRobotOffset
                + "\npin offset: " + pinOffset + ", pin offset (flipped): " + flippedPinOffset
                + "\n" + "-".repeat(20)
            );
        }
    }

    /**
     * Set the "robot"-relative swerve drive powers of the subsystem.
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction.
     * @param yPower The power [-1.0, 1.0] in the y (left) direction.
     */
    public void setDrivePowers(double xPower, double yPower) {
        // Scale [-1.0, 1.0] powers to desired velocity, turning field-relative powers into 
        // robot relative chassis speeds. For the missile's single-module setup, we assume that 
        // angular power is always 0 and the "robot" is always facing forward.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * MAX_VEL, 
            yPower * MAX_VEL, 
            0,
            new Rotation2d()
        );

        // Calculate swerve module states from desired chassis speeds.
        this.states = kinematics.toSwerveModuleStates(speeds);
    }

    /**
     * Sets the swerve module states of this subsystem from provided field-centric
     * swerve drive powers.
     * 
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction.
     */
    @Override
    public void setDrivePowers(double xPower) {
        setDrivePowers(xPower, 0.0);
    }
}
