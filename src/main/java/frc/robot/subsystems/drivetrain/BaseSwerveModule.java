package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface BaseSwerveModule {
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

    /**
     * Sets whether the integrated relative encoder should be used as the feedback source
     * for steer closed-loop control. This method is a no-op if the implementing swerve module
     * does not support feedback device swapping.
     * 
     * @param useRelative Whether to use relative encoders for feedback control. If `false`, uses the external absolute encoder instead.
     */
    public default void setSteerRelativeFeedback(boolean useRelative) {};
}
