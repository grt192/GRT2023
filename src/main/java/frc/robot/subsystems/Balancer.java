package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Balancer extends SubsystemBase {

    public int phase;
    public int initialHeading;

    double returnPower; //power to be returned to DT
    double oldAngle; // logging previous pitch angle for use in controller
    double currentAngle; // current pitch angle
    double gain = 0.01; // gain for controller

    private final AHRS ahrs; 

  public Balancer() {
    ahrs = new AHRS(SPI.Port.kMXP);
    initialHeading = (int) ahrs.getCompassHeading();

  }

  public double calcPower(){
    
    // move forward slowly until angle increases
    // start proportional controller until angle is within 2 degrees
    // kill power

    switch (phase) {
        case 1:
            if((ahrs.getPitch() >= 9.0)){
                phase ++; // increase the stage number if stage 1 completed and conditions met
            }
            else{
                returnPower = 0.1; // drive robot slowly (towards the station)
            }
            break;
        case 2:
            
            // proportional-ish controller
            currentAngle = ahrs.getPitch();
            // double angleDelta  = currentAngle - oldAngle; // figure out the change in angle
            if(Math.abs(currentAngle) <= 2.0){ // if the angle is within 2 degrees of level
                phase++;
            }
            else{
                returnPower += gain * currentAngle; // the desired angle is level, so the current angle of the robot is inherently the error
            }
                        
            break;
        
        case 3:
            // if(stage3()){
            //     // kill
            // }
            // else{
            //     // constant value
            // }
            returnPower = 0.0;

            break;
    
        default:
            break;
    }

    return returnPower;


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}