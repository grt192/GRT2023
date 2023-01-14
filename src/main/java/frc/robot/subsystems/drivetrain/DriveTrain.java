package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public  class Drivetrain extends SubsystemBase {
    public final AHRS ahrs;
    
    
    double xPower;
    double yPower;
    double angularPower;

    Drivetrain(){
        ahrs = new AHRS(SPI.Port.kMXP);

    }

    public void updateDrivePowers(double forwardComponent, double sideComponent, double angularPower){
        // xPower = forwardComponent;
        // yPower = sideComponent;
        // this.angularPower = angularPower;
        // System.out.println("dt received drive powers");
    }

}
