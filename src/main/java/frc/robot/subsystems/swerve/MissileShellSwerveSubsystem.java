package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A shell swerve subsystem to run a single swerve module on the missile.
 */
public class MissileShellSwerveSubsystem extends SubsystemBase {
    private final SwerveModule module;
    private final SwerveDriveKinematics kinematics;

    public static final double MAX_VEL = 1.0; // Max robot tangential velocity, in percent output

    private SwerveModuleState[] states = {
        new SwerveModuleState()
    };
    private boolean isIdle = false;

    public MissileShellSwerveSubsystem() {
        module = new SwerveModule(2, 1, -1.1350287199020386 + Math.PI);

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
    }

    /**
     * Set the field-centric swerve drive powers of the subsystem.
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction.
     * @param yPower The power [-1.0, 1.0] in the y (left) direction.
     * @param angularPower The angular (rotational) power [-1.0, 1.0].
     */
    public void setSwerveDrivePowers(double xPower, double yPower, double angularPower) {
        // If drivers are sending no input, we're idle; don't return modules to 0 degrees.
        if (xPower == 0.0 && yPower == 0.0 && angularPower == 0.0) {
            if (isIdle) return;

            // Stop all modules but hold their current angle.
            SwerveModuleState state = module.getState();
            this.states[0] = new SwerveModuleState(0.0, state.angle);

            isIdle = true;
            return;
        } else {
            isIdle = false;
        }

        // Scale [-1.0, 1.0] powers to desired velocity, turning field-relative powers
        // into robot relative chassis speeds.
        // For the missile's single-module setup, we assume that angular power is always 0 and
        // the "robot" is always facing forward.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * MAX_VEL, 
            yPower * MAX_VEL, 
            0,
            new Rotation2d()
        );

        // Calculate swerve module states from desired chassis speeds.
        this.states = kinematics.toSwerveModuleStates(speeds);
    }
}
