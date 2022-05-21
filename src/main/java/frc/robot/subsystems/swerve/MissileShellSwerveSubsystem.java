package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.shuffleboard.GRTShuffleboardTab;

public class MissileShellSwerveSubsystem extends SubsystemBase {
    private final NEOTalonSwerveModule module;

    private final SwerveDriveKinematics kinematics;

    public static final double MAX_VEL = 1.0; // Max robot tangential velocity, in percent output

    private final GRTShuffleboardTab shuffleboardTab = new GRTShuffleboardTab("Swerve");
    private final GRTNetworkTableEntry driveVelEntry, steerAngleEntry;

    public MissileShellSwerveSubsystem() {
        module = new NEOTalonSwerveModule(9, 2);

        // One module at the center of the robot
        kinematics = new SwerveDriveKinematics(
            new Translation2d(),
            new Translation2d()
        );

        driveVelEntry = shuffleboardTab.addEntry("Drive vel", 0).at(0, 0);
        steerAngleEntry = shuffleboardTab.addEntry("Steer angle", 0).at(0, 1);
    }

    @Override
    public void periodic() {
        driveVelEntry.setValue(module.getDriveVelocity());
        steerAngleEntry.setValue(module.getState().angle.getDegrees());
    }

    /**
     * Set the field-centric swerve drive powers of the subsystem.
     * @param xPower The power [-1.0, 1.0] in the x (forward) direction.
     * @param yPower The power [-1.0, 1.0] in the y (left) direction.
     * @param angularPower The angular (rotational) power [-1.0, 1.0].
     */
    public void setSwerveDrivePowers(double xPower, double yPower, double angularPower) {
        // Scale [-1.0, 1.0] powers to desired velocity, turning field-relative powers
        // into robot relative chassis speeds.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * MAX_VEL, 
            yPower * MAX_VEL, 
            0,
            new Rotation2d()
        );

        // Calculate swerve module states from desired chassis speeds, desaturating them to
        // ensure all velocities are under MAX_VEL after kinematics.
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VEL);

        module.setDesiredState(states[0]);
    }
}
