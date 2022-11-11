

package frc.robot.subsystems.tank;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import java.lang.Math.*;


public class Tank extends SubsystemBase{
    
    private final WPI_TalonSRX leftmotor = new WPI_TalonSRX(1);
    private final WPI_TalonSRX leftmotor2 = new WPI_TalonSRX(2);
    private final WPI_TalonSRX rightmotor = new WPI_TalonSRX(3);
    private final WPI_TalonSRX rightmotor2 = new WPI_TalonSRX(4);
    private final double MAXSPEED = .5;

    public double forwardpower = 0;
    public double turnpower = 0;
    

    public Tank(){
        leftmotor.configFactoryDefault();
        leftmotor2.configFactoryDefault();
        rightmotor.configFactoryDefault();
        rightmotor2.configFactoryDefault();

        leftmotor.setNeutralMode(NeutralMode.Brake);
        leftmotor2.setNeutralMode(NeutralMode.Brake);
        rightmotor.setNeutralMode(NeutralMode.Brake);
        rightmotor2.setNeutralMode(NeutralMode.Brake);

        leftmotor2.follow(leftmotor);
        rightmotor2.follow(rightmotor);

        leftmotor.setInverted(true);
    }

    @Override
    public void periodic() {
        //Powers updated in RobotContainer periodic, sent to motors here
        //Divided so that the motor powers will never exceed MAXSPEED (or negative MAXSPEED)
        //(Basically just scales the motors when turning)


        // System.out.println(forwardpower);
        leftmotor.set(MAXSPEED * (forwardpower - turnpower)/(1 + Math.abs(turnpower)));
        rightmotor.set(MAXSPEED * (forwardpower + turnpower)/(1 + Math.abs(turnpower)));
    }
}
