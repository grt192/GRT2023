package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.util.MotorUtil;
import static frc.robot.Constants.TankConstants.*;

public class TankIOReal implements TankIO {
    private final WPI_TalonSRX leftMain;
    private final WPI_TalonSRX leftFollow;

    private final WPI_TalonSRX rightMain;
    private final WPI_TalonSRX rightFollow;

    public TankIOReal() {
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
    public void setLeftPower(double power) {
        leftMain.set(power);
    }

    @Override
    public void setRightPower(double power) {
        rightMain.set(power);
    }
}
