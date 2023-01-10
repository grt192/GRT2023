package frc.robot.subsystems;

import static frc.robot.Constants.TankConstants.LEFT_MAIN;
import static frc.robot.Constants.TankConstants.LEFT_SECONDARY;
import static frc.robot.Constants.TankConstants.RIGHT_MAIN;
import static frc.robot.Constants.TankConstants.RIGHT_SECONDARY;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // internal notfalcon CIM motors

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;

public class Tank extends SubsystemBase {
    
    public boolean driveInvert;

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

    leftDrive = forwardComponent + sideComponent*0.75;
    rightDrive = forwardComponent - sideComponent*0.75;


    if(Math.abs(leftDrive) >= 1.0){
      leftDrive = leftDrive / Math.abs(leftDrive);
      rightDrive = rightDrive / Math.abs(leftDrive);
    }
    if(Math.abs(rightDrive) >= 1.0){
        rightDrive = rightDrive / Math.abs(rightDrive);
        leftDrive = leftDrive / Math.abs(rightDrive);
    }
    
    if(driveInvert){
      leftDrive = leftDrive * -1;
      rightDrive = rightDrive * -1;
    }

    left.set(leftDrive);
    right.set(rightDrive);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}