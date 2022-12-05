// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // internal notfalcon CIM motors
import frc.robot.motorcontrol.MotorUtil;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController; // controller input library

import static frc.robot.Constants.TankConstants.*;

public class Tank extends SubsystemBase {
    
    private final WPI_TalonSRX left = MotorUtil.createTalonSRX(LEFT_MAIN); // left motor
    private final WPI_TalonSRX left2 = MotorUtil.createTalonSRX(LEFT_SECONDARY); // left motor

    private final WPI_TalonSRX right = MotorUtil.createTalonSRX(RIGHT_MAIN); // right motor
    private final WPI_TalonSRX right2 = MotorUtil.createTalonSRX(RIGHT_SECONDARY); // right motor

    public double sideComponent;
    public double forwardComponent;
  
    double leftDrive;
    double rightDrive;

  public Tank() {

    left2.follow(left);
    right2.follow(right);

    left.setNeutralMode(NeutralMode.Brake);
    right.setNeutralMode(NeutralMode.Brake);
    right.setInverted(true);
    right2.setInverted(true);


  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    leftDrive = forwardComponent + sideComponent;
    rightDrive = forwardComponent - sideComponent;

    if(Math.abs(leftDrive) >= 1.0){
      leftDrive = leftDrive / Math.abs(leftDrive);
      rightDrive = rightDrive / Math.abs(leftDrive);
    }
    if(Math.abs(rightDrive) >= 1.0){
        rightDrive = rightDrive / Math.abs(rightDrive);
        leftDrive = leftDrive / Math.abs(rightDrive);
    }

    left.set(leftDrive);
    right.set(rightDrive);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}