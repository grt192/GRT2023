// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // internal Falcon motors

import edu.wpi.first.wpilibj.XboxController; // controller input library

public class Tank extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Tank() {

    left2.follow(left);
    right2.follow(right);
  }
  
  private final WPI_TalonSRX left = new WPI_TalonSRX​(1); // left motor
  private final WPI_TalonSRX left2 = new WPI_TalonSRX​(2); // left motor
  

  private final WPI_TalonSRX right = new WPI_TalonSRX​(3); // right motor
  private final WPI_TalonSRX right2 = new WPI_TalonSRX​(4); // right motor


  private final XboxController controller = new XboxController(0); // controller

  double sideComponent;
  double forwardComponent;
  
  double leftDrive;
  double rightDrive;

  boolean inactive; // sticks centered --> stationary
  boolean skid; // yaw only --> skid steer in place
  boolean drive;  // drive only --> front/back motion
  boolean bank; // skid/drive at the same time



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    forwardComponent = controller.getLeftY();
    sideComponent = controller.getRightX();

    skid = (sideComponent != 0.0) && (forwardComponent == 0.0);
    drive = (sideComponent == 0.0) && (forwardComponent != 0.0);
    inactive = (sideComponent == 0.0) && (forwardComponent == 0.0);

    if(inactive){
        left.set(0.0);
        right.set(0.0);
    }
    if(drive){
        left.set(forwardComponent);
        right.set(forwardComponent);
    }
    if(skid){
        left.set(sideComponent);
        right.set(sideComponent * -1);
    }
    if(bank){
        
        leftDrive = (sideComponent + forwardComponent) / 2.0;
        rightDrive = (forwardComponent - sideComponent) / 2.0;
        
        left.set(leftDrive);
        right.set(rightDrive);
    }

    


    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}