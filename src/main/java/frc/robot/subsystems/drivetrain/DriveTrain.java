package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Drivetrain extends SubsystemBase {

    double xPower;
    double yPower;
    double angularPower;

    public void updateDrivePowers(double forwardComponent, double sideComponent, double angularPower){
        // xPower = forwardComponent;
        // yPower = sideComponent;
        // this.angularPower = angularPower;
        // System.out.println("dt received drive powers");
    }

}
