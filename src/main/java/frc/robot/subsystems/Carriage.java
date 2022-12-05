// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // internal talon motors

import edu.wpi.first.wpilibj.XboxController; // controller input 

import com.revrobotics.CANSparkMax; // front intake motor
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;

import static frc.robot.Constants.CarriageConstants.*;


public class Carriage extends SubsystemBase {
    
    public boolean openDoor = false; // tracks whether door should be open or closed
    public boolean liftCarriage = false;  // tracks whether carriage should be lifted

    DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CARRIAGE_SOLENOID_F,CARRIAGE_SOLENOID_R); // lifts carriage
    
    Servo top = new Servo(TOP_SERVO); // top servo opening door
    Servo bottom = new Servo(BOTTOM_SERVO); // bottom servo opening door

  public Carriage() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
    if(liftCarriage){
      piston.toggle();
      liftCarriage = false;

    }
    

    if(openDoor){
        top.set(TOP_OPEN);
        bottom.set(BOTTOM_OPEN);
    }
    else{
        top.set(TOP_CLOSED);
        bottom.set(BOTTOM_CLOSED);
    }

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}