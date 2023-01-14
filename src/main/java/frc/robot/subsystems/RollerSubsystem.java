package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;

import static frc.robot.Constants.RollerConstants.*;

public class RollerSubsystem extends SubsystemBase{
    
    private final WPI_TalonSRX leftbeak = MotorUtil.createTalonSRX(LEFTID);
    private final WPI_TalonSRX rightbeak = MotorUtil.createTalonSRX(RIGHTID);
    
    //limit switch
    private final DigitalInput crimitswitch = new DigitalInput(0);
    //timer
    public Timer timere = new Timer();
    //roller state 
    public String rollstate;

    //constructor
    public RollerSubsystem(){
        rollstate = "stop";
        rightbeak.follow(leftbeak);
    }

    //toggles roll state when B button is pressed
    public void RollToggle(){
        if (rollstate == "in"){
            rollstate = "out";
        }
        else{
            rollstate = "in";
        }
    }

    @Override
    public void periodic() {
        //if wheels must intake, and the limit switch is not pressed, turn on motors
        if (rollstate == "in" && !crimitswitch.get()){
            leftbeak.set(.5);
        }
        //if wheels must place, turn on timer then turn on motors for 2 seconds, reset timer
        else if (rollstate == "out"){
            timere.start();
            while (timere.get() < 2000){
                leftbeak.set(-.5);
            }
            rollstate = "stop";
            timere.reset();
        }
        //if limit switch is hit, or place has placed, keep motor off
        else{
            leftbeak.set(0);
        }
        
    }






}
