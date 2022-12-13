// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.FRONT_MOTOR;
import static frc.robot.Constants.IntakeConstants.FRONT_OPP_MOTOR;
import static frc.robot.Constants.IntakeConstants.LEFT_MOTOR;
import static frc.robot.Constants.IntakeConstants.RIGHT_MOTOR;

import com.revrobotics.CANSparkMax; // front intake motor
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;



public class Intake extends SubsystemBase {

  public boolean bondMode;
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
    //intake_power = intake_power_forward + intake_power_reverse;

    intake_power = 0.0; // reset intake power value (so it doesnt compound and go beserk (#learninglessons))

    if(intake_power_forward <= 0.10){ // if forward intake trigger is not pressed significantly, dont make it spin
      intake_power += 0.0;
    }
    if(intake_power_forward > 0.10 && intake_power_forward < 0.85){ // if its pressed a little, make it spin slow
      intake_power += 0.18;
    }
    if(intake_power_forward > 0.85){
      intake_power += 0.35; // if its pressed a lot, make it spin fast
    }
    if(intake_power_reverse >= 0.4){ // if there is some reverse component, make it spin that way 
      intake_power -= 0.45;
    }
    
      front.set(intake_power);
      main_roller.set(intake_power * 0.75); 
  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}