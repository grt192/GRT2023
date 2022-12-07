// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.FRONT_MOTOR;
import static frc.robot.Constants.IntakeConstants.FRONT_OPP_MOTOR;
import static frc.robot.Constants.IntakeConstants.LEFT_MOTOR;
import static frc.robot.Constants.IntakeConstants.RIGHT_MOTOR;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // internal talon motors
import com.revrobotics.CANSparkMax; // front intake motor
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;



public class Intake extends SubsystemBase {

  public boolean intake_off = false;
  public double intake_power;
  public double intake_power_forward;
  public double intake_power_reverse;

  private final CANSparkMax front = MotorUtil.createSparkMax(FRONT_MOTOR); // front motor for intake
  private final CANSparkMax frontOpposite = MotorUtil.createSparkMax(FRONT_OPP_MOTOR);// motor opposite front motor for intake
  
  private final CANSparkMax main_roller = MotorUtil.createSparkMax(RIGHT_MOTOR); // right roller between intake and carriage
  private final CANSparkMax opp_roller = MotorUtil.createSparkMax(LEFT_MOTOR); // left roller between intake and carriage

  public Intake() {

    front.setIdleMode(IdleMode.kCoast);
    front.setInverted(true);
    frontOpposite.setIdleMode(IdleMode.kCoast);
    opp_roller.setIdleMode(IdleMode.kCoast);
    main_roller.setIdleMode(IdleMode.kCoast);

    
    frontOpposite.follow(front);
    opp_roller.follow(main_roller);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // create an intake_power from the forward/reverse intake components
    // both shouldn't need to be pressed at the same time, so one of them will likely be 0 anyways
    intake_power = intake_power_forward + intake_power_reverse;

    // if intake needs to be disabled for whatever reason (more disabled than just not pressing anything)
    if(intake_off){
        front.set(0.0);
        main_roller.set(0.0);   
    }
    else{
      front.set(intake_power);
      main_roller.set(intake_power); 
      // assuming the rollers should be set to match the intake as opposed to being set to a fixed value
    }
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}