// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CarriageConstants.BOTTOM_CLOSED;
import static frc.robot.Constants.CarriageConstants.BOTTOM_OPEN;
import static frc.robot.Constants.CarriageConstants.BOTTOM_SERVO;
import static frc.robot.Constants.CarriageConstants.CARRIAGE_SOLENOID_F;
import static frc.robot.Constants.CarriageConstants.CARRIAGE_SOLENOID_R;
import static frc.robot.Constants.CarriageConstants.TOP_CLOSED;
import static frc.robot.Constants.CarriageConstants.TOP_OPEN;
import static frc.robot.Constants.CarriageConstants.TOP_SERVO;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Carriage extends SubsystemBase {
    
    public boolean openDoor = true; // tracks whether door should be open or closed
    public boolean liftCarriage = false;  // tracks whether carriage should be lifted
    public boolean carriageEject;
    public boolean timer_enabled;
    DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CARRIAGE_SOLENOID_F,CARRIAGE_SOLENOID_R); // lifts carriage
    Timer timer = new Timer();
    Servo top = new Servo(TOP_SERVO); // top servo opening door
    Servo bottom = new Servo(BOTTOM_SERVO); // bottom servo opening door

  public Carriage() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(carriageEject){ // if we need to eject something from above the carriage
      piston.set(kForward); // note that this mode may lead to good blocks being ejected as well
      carriageEject = false;
    }

    if(liftCarriage){ //lifting carriage for normal operation
       
      if(!timer_enabled){ //if the timer was not enabled, close the door and then start the timer
        top.setAngle(TOP_CLOSED);
        bottom.setAngle(BOTTOM_CLOSED);
        openDoor = false;
        timer.reset(); // reset and start timer to make sure doors are closed before lift
        timer.start();
        timer_enabled = true;
      }
      if(timer.get() >= 0.2){ // if 0.2 seconds pass, set the piston forward
        piston.set(kForward);
        timer.stop();
      }
      
    }
    else{ // if liftcarriage not true (shouldn't be lifting/lifted), set the piston to the down position 
      piston.set(kReverse);
      if(timer_enabled){ // timer_enabled is true from the liftcarriage until this point 
        openDoor = true; // this allows us to automatically open the servo door so we don't forget to
      }
      timer_enabled = false;
    }

    if(openDoor){
      top.setAngle(TOP_OPEN);
      bottom.setAngle(BOTTOM_OPEN);
    }
    else{
      top.setAngle(TOP_CLOSED);
      bottom.setAngle(BOTTOM_CLOSED);
    }

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}