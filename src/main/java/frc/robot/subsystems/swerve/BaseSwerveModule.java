package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface BaseSwerveModule {
    public SwerveModulePosition getState();
    public void setDesiredState(SwerveModuleState state);
}
