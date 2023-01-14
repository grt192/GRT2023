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

    // TODO: account for tank drive 2-power setup
    public abstract void setDrivePowers(double forwardPower, double sidePower, double angularPower);

    /**
     * Gets the NavX AHRS on this drivetrain.
     * @return The NavX object.
     */
    public AHRS getAhrs() {
        return ahrs;
    }
}
