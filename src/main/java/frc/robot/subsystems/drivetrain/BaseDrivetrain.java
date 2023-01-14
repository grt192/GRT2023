package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

/**
 * The superclass of all drivetrain subsystems (both tank and swerve).
 */
public abstract class BaseDrivetrain extends SubsystemBase {
    protected final AHRS ahrs;

    public BaseDrivetrain() {
        ahrs = new AHRS(SPI.Port.kMXP);
    }

    /**
     * Sets the forward power of this drive system. Subclasses should have separate methods for
     * more advanced control (ie. tank setting forward and turn powers, swerve setting x, y, and turn 
     * powers, etc.) and this method should call those with default parameters.
     * 
     * @param forwardPower The power [-1.0, 1.0] in the forward direction.
     */
    public abstract void setDrivePowers(double forwardPower);

    /**
     * Gets the NavX AHRS on this drivetrain.
     * @return The NavX object.
     */
    public AHRS getAhrs() {
        return ahrs;
    }
}
