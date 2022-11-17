// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // internal Falcon motors

import edu.wpi.first.wpilibj.XboxController; // controller input 

import com.revrobotics.CANSparkMax; // front intake motor
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.CarriageConstants.*;

public class Carriage extends SubsystemBase {
    
    boolean openDoor = false; // tracks whether door should be open or closed
    boolean liftCarriage = false;  // tracks whether carriage should be lifted

    int open = 90; // servo position when the door is open
    int closed = 0; // servo position when the door is closed

    Solenoid piston = new Solenoid(PneumaticsModuleType.CTREPCM, carriage_solenoid); // lifts carriage
    
    Servo top = new Servo(top_servo); // top servo opening door
    Servo bottom = new Servo(bottom_servo); // bottom servo opening door

    private final XboxController controller = new XboxController(0); // controller

  public Carriage() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(controller.getRightBumper()){
        liftCarriage = !liftCarriage; // toggle switch (true --> false or false --> true)
    }

    if(controller.getLeftBumper()){
        openDoor = !openDoor;
    }

    piston.set(liftCarriage);

    if(openDoor){
        top.set(open);
        bottom.set(open);
    }
    else{
        top.set(closed);
        bottom.set(closed);
    }

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}