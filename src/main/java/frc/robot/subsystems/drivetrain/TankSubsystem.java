package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.motorcontrol.MotorUtil;

import static frc.robot.Constants.TankConstants.*;

public class TankSubsystem extends BaseDrivetrain {
    private final WPI_TalonSRX leftMain;
    private final WPI_TalonSRX leftFollow;

    private final WPI_TalonSRX rightMain;
    private final WPI_TalonSRX rightFollow;

    private double leftDrive;
    private double rightDrive;

    public TankSubsystem() {
        leftMain = MotorUtil.createTalonSRX(LEFT_MAIN);
        leftMain.setNeutralMode(NeutralMode.Brake);

        leftFollow = MotorUtil.createTalonSRX(LEFT_FOLLOW);
        leftFollow.follow(leftMain);

        rightMain = MotorUtil.createTalonSRX(RIGHT_MAIN);
        rightMain.setNeutralMode(NeutralMode.Brake);
        rightMain.setInverted(true);

        rightFollow = MotorUtil.createTalonSRX(RIGHT_FOLLOW);
        rightFollow.follow(rightMain);
        rightFollow.setInverted(InvertType.FollowMaster);
    }

    @Override
    public void setDrivePowers(double forwardComponent, double sideComponent, double angularPower) {
        // POSSIBLE/CONCEPT -- double theta = Math.atan(yPower/xPower); // if the robot needs to move some amount laterally, it can rotate to this angle and move in that direction

        // System.out.println("Calculating DT power");
        leftDrive = forwardComponent + sideComponent;
        rightDrive = forwardComponent - sideComponent;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (Math.abs(leftDrive) >= 1.0) {
            leftDrive = leftDrive / Math.abs(leftDrive);
            rightDrive = rightDrive / Math.abs(leftDrive);
        }
        if (Math.abs(rightDrive) >= 1.0) {
            rightDrive = rightDrive / Math.abs(rightDrive);
            leftDrive = leftDrive / Math.abs(rightDrive);
        }

        leftMain.set(leftDrive);
        rightMain.set(rightDrive); 
        // System.out.println("Set Drive Powers.");
    }
}
