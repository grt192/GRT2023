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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Carriage extends SubsystemBase {
    
    public boolean openDoor = false; // tracks whether door should be open or closed
    public boolean liftCarriage = false;  // tracks whether carriage should be lifted

    DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CARRIAGE_SOLENOID_F,CARRIAGE_SOLENOID_R); // lifts carriage
    Timer timer = new Timer();
    Servo top = new Servo(TOP_SERVO); // top servo opening door
    Servo bottom = new Servo(BOTTOM_SERVO); // bottom servo opening door

  public Carriage() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
    if(liftCarriage){ 
      top.setAngle(TOP_CLOSED); // if set to lift carriage, first close the doors
      bottom.setAngle(BOTTOM_CLOSED);
      timer.reset(); // reset and start timer to make sure doors are closed before lift
      timer.start();
      liftCarriage = false; // set to false so we don't keep resetting timer
      
    }
    else{ // if not to lift carriage, set the door to be open
      top.setAngle(TOP_OPEN);
      bottom.setAngle(BOTTOM_OPEN);
    }

    if(timer.get() >= 0.2){ //if time elapsed meets criterion, 
      timer.stop();
      timer.reset();
      piston.toggle();
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