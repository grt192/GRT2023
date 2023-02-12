package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface BaseSwerveModule extends AutoCloseable {
    /**
     * Gets the current state of the module as a `SwerveModulePosition`.
     * @return The state of the module.
     */
    public SwerveModulePosition getState();

    /**
     * Sets the desired state of the module.
     * @param state The desired state of the module as a `SwerveModuleState`.
     */
    public void setDesiredState(SwerveModuleState state);
}
