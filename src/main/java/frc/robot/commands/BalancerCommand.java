package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class BalancerCommand extends CommandBase {

  private final BaseSwerveSubsystem swerveSubsystem;
  public int phase = 1;
    double returnPower; //power to be returned to DT
    double currentAngle; // current pitch angle
    double gain = 0.01; // gain for controller
    double initialHeading;

    private AHRS ahrs; 


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveSubsystem2 The subsystem used by this command.
   */
  public BalancerCommand(BaseSwerveSubsystem swerveSubsystem2,AHRS ahrs) {
    swerveSubsystem = swerveSubsystem2;
    this.ahrs = ahrs;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialHeading = ahrs.getCompassHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (phase) {
        case 1:
            if((ahrs.getPitch() >= 9.0)){
                returnPower = 0.0;
                phase ++; // increase the stage number if stage 1 completed and conditions met
            }
            else{
                returnPower = 0.1; // drive robot slowly (towards the station)
            }
            break;
        case 2:
            
            // proportional-ish controller
            currentAngle = ahrs.getPitch();
            if(Math.abs(currentAngle) <= 2.0){ // if the angle is within 2 degrees of level
                phase++;
            }
            else{
                returnPower += gain * currentAngle; // the desired angle is level, so the current angle of the robot is inherently the error
            }
                        
            break;
        
        case 3:
            returnPower = 0.0;

            break;
    
        default:
            break;
    }
    swerveSubsystem.setSwerveDrivePowers(returnPower,0.0,initialHeading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == 3;
  }
}