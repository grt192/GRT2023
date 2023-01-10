package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Balancer extends SubsystemBase {

    public int step;

    double returnPower;
    double oldAngle;
    double currentAngle;
    double gain = 0.01;

    private final AHRS ahrs;

  public Balancer() {
    ahrs = new AHRS(SPI.Port.kMXP);

  }

  private boolean stage1(){
    // if the angle increases
    if(ahrs.getPitch() >= 15.0){
        oldAngle = ahrs.getPitch();
        return true;
    }
    else{
        return false;
    }
  }

  private boolean stage2(){
    // start proportional controller until angle is within 2 degrees
    return true;
    



  }

  private boolean stage3(){
    // kill power
    return true;


  }


  public double calcPower(){
    
    // move forward slowly until angle increases
    // start proportional controller until angle is within 2 degrees
    // kill power


    switch (step) {
        case 1:
            if(stage1()){
                step ++; // increase the stage number
            }
            else{
                returnPower = 0.1; // drive robot slowly towards the station
            }
            break;
        case 2:
            if(stage2()){
                step ++;
            }
            else{
                // proportional-ish controller
                currentAngle = ahrs.getPitch();
                double angledelta  = currentAngle - oldAngle;
                returnPower += gain * angledelta;
            }
            break;
        
        case 3:
            if(stage3()){
                // kill
            }
            else{
                // constant value
            }
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