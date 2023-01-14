package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

/** An example command that uses an example subsystem. */
public class BalancerCommand extends CommandBase {

  private Drivetrain driveTrain;

  public int phase = 1;
  double returnPower; //power to be returned to DT
  double currentAngle = 0; // current pitch angle
  double oldAngle = 0;
  double deltaAngle;
  double gain = 0.01; // gain for controller
  double initialHeading;

  private AHRS ahrs = driveTrain.ahrs; 

  public BalancerCommand(Drivetrain subsystem) {
    driveTrain = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialHeading = ahrs.getCompassHeading();
    System.out.println("balancer initialized");
    oldAngle = -ahrs.getPitch();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("pitch");
    System.out.println(-1*ahrs.getPitch());

    switch (phase) {
        case 1: // moving forward unless delta angle is NEGATIVE (robot pitches down a little bit once most of the mass is on the station)
            currentAngle = -ahrs.getPitch();
            deltaAngle = currentAngle - oldAngle;
            if((deltaAngle <= -0.4)){
                returnPower = 0.0;
                phase ++; // increase the stage number if stage 1 completed and conditions met
            }
            else{ 
                System.out.println("auton speed");
                returnPower = 0.3; // drive robot slowly (towards the station)
                oldAngle = currentAngle;
            }
            break;
        case 2:
            
            // proportional-ish controller
            currentAngle = -ahrs.getPitch();
            if(Math.abs(currentAngle) <= 2.0){ // if the angle is within 2 degrees of level
              returnPower = 0.0;  
              phase++;
            }
            else{
                returnPower = gain * currentAngle; // the desired angle is level, so the current angle of the robot is inherently the error
                Math.min(0.25,returnPower);
            }
                        
            break;
        
        case 3: // stops the robot if it thinks its level, then checks again
            returnPower = 0.0;
            currentAngle = -ahrs.getPitch();
            if(Math.abs(currentAngle) <= 2.0){ // if the angle is within 2 degrees of level
              returnPower = 0.0;  
              phase++;
            }
            else{
                returnPower = gain * currentAngle; // the desired angle is level, so the current angle of the robot is inherently the error
                Math.min(0.25,returnPower);
            }
            break;
    
        default:
            break;
    }
    driveTrain.updateDrivePowers(returnPower,0.0,initialHeading);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("----------- Balancing process finished ---------------------");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return phase == 4;
  }
}