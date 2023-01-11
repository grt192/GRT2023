package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractSwerveSubsystem extends SubsystemBase {
    private final AHRS ahrs;

    public AbstractSwerveSubsystem() {
        ahrs = new AHRS(SPI.Port.kMXP);
    }

    private double angleoffset = 0;

    public abstract void setSwerveDrivePowers(double xPower, double yPower, double angularPower);

    public void resetFieldAngle() {
        angleoffset = ahrs.getAngle();
    }

    public Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

    public Rotation2d getGyroForField() {
        return Rotation2d.fromDegrees(-(ahrs.getAngle() - angleoffset));
    }

}
