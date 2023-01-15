package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    //public Timer timere = new Timer();

    //roller state 
    public double rollstate;

    //constructor
    public RollerSubsystem(){
        rollstate = 0.;
        rightbeak.follow(leftbeak);
        leftbeak.setInverted(true);
        leftbeak.setNeutralMode(NeutralMode.Brake);
        rightbeak.setNeutralMode(NeutralMode.Brake);
        //rightbeak.setInverted(true);

    }

    //toggles roll state when B button is pressed
    // public void rollToggle(){
    //     if (rollstate == Rollstate.in){
    //         rollstate = Rollstate.out;
    //     }
    //     else{
    //         rollstate = Rollstate.in;
    //     }
    // }

    @Override
    public void periodic() {
        //if wheels must intake, and the limit switch is not pressed, turn on motors
        if (crimitswitch.get()){
            leftbeak.set(rollstate);
            System.out.println("l");
        }
        //if wheels must place, turn on timer then turn on motors for 2 seconds, reset timer
        //if limit switch is hit, or place has placed, keep motor off
        else{
            if(rollstate <= 0.0){
                leftbeak.set(rollstate);
            }
            else{
                leftbeak.set(0);
            }
            
        }
        
    }






}
