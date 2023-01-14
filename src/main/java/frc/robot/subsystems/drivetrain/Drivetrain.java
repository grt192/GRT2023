package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {
    public final AHRS ahrs;

    public Drivetrain() {
        ahrs = new AHRS(SPI.Port.kMXP);
    }

    public void setDrivePowers(double forwardComponent, double sideComponent, double angularPower) {}
}