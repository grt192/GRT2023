// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // internal talon motors
import frc.robot.motorcontrol.MotorUtil;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController; // controller input 

import com.revrobotics.CANSparkMax; // front intake motor
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Servo;

import static frc.robot.Constants.IntakeConstants.*;


public class Intake extends SubsystemBase {

  public boolean intake_down = false;
  public boolean intake_on = false;
  public double intake_power;

  private final CANSparkMax front = MotorUtil.createSparkMax(FRONT_MOTOR); // front motor for intake
  private final CANSparkMax frontOpposite = MotorUtil.createSparkMax(FRONT_OPP_MOTOR);// motor opposite front motor for intake
  
  private final WPI_TalonSRX main_roller = MotorUtil.createTalonSRX(RIGHT_MOTOR); // right roller between intake and carriage
  private final WPI_TalonSRX opp_roller = MotorUtil.createTalonSRX(LEFT_MOTOR); // left roller between intake and carriage

  public Intake() {

    front.setIdleMode(IdleMode.kBrake);
    frontOpposite.setIdleMode(IdleMode.kBrake);
    opp_roller.setNeutralMode(NeutralMode.Brake);
    main_roller.setNeutralMode(NeutralMode.Brake);

    frontOpposite.setInverted(true);
    frontOpposite.follow(front);
    opp_roller.setInverted(true);
    opp_roller.follow(main_roller);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(intake_on){
        front.set(intake_power);
        main_roller.set(0.5);   
    }
    else{
        front.set(0.0);
        main_roller.set(0.0);
    }

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}