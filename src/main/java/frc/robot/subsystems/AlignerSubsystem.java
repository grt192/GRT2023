package frc.robot.subsystems;

import static frc.robot.Constants.AlignerConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.motorcontrol.MotorUtil;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class AlignerSubsystem {
    private final WPI_TalonSRX motorSlapper = MotorUtil.createTalonSRX(slapId);
    private final WPI_TalonSRX motorAngler = MotorUtil.createTalonSRX(angleId);

    public double state = 0;

    public AlignerSubsystem() {
        motorSlapper.setNeutralMode(NeutralMode.Brake);
        motorAngler.setNeutralMode(NeutralMode.Brake);

    }
    public void periodic(){
        

    }

}
