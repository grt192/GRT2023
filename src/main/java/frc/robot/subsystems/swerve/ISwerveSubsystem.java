package frc.robot.subsystems.swerve;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class ISwerveSubsystem extends SubsystemBase{
    public abstract void resetFieldAngle();
    public abstract void setSwerveDrivePowers(double xPower, double yPower, double angularPower);
}

