package frc.robot.subsystems;

import static frc.robot.Constants.AlignerConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.shuffleboard.GRTShuffleboardTab;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class AlignerSubsystem extends SubsystemBase {
    private final WPI_TalonSRX motorSlapper = MotorUtil.createTalonSRX(slapId);
    private final WPI_TalonSRX motorAngler = MotorUtil.createTalonSRX(angleId);
    private final GRTShuffleboardTab shuffleboardTab = new GRTShuffleboardTab("Aligner");
    private final GRTNetworkTableEntry positionEntry = shuffleboardTab.addEntry("Slapperposition", 0).at(0, 0); 

    public double state = 0;
    public double slapper;
    public double angler;

    public AlignerSubsystem() {
        motorSlapper.setNeutralMode(NeutralMode.Brake);
        motorAngler.setNeutralMode(NeutralMode.Brake);

        motorSlapper.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motorSlapper.setSensorPhase(true);

    }
    public void periodic(){
        motorSlapper.set(.5);
        positionEntry.setValue(motorSlapper.getSelectedSensorPosition());

    }

}
