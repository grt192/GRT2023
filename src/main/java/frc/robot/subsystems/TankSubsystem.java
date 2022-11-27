
package frc.robot.subsystems;

import static frc.robot.Constants.TankConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorcontrol.MotorUtil;

public class TankSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftmotor = MotorUtil.createTalonSRX(MOTOR_BACKLEFT);
    private final WPI_TalonSRX leftmotor2 = MotorUtil.createTalonSRX(MOTOR_FRONTLEFT);
    private final WPI_TalonSRX rightmotor = MotorUtil.createTalonSRX(MOTOR_BACKRIGHT);
    private final WPI_TalonSRX rightmotor2 = MotorUtil.createTalonSRX(MOTOR_FRONTRIGHT);

    public double forwardpower = 0;
    public double turnpower = 0;
    public double scaler;

    public TankSubsystem() {


        leftmotor.setNeutralMode(NeutralMode.Brake);
        leftmotor2.setNeutralMode(NeutralMode.Brake);
        rightmotor.setNeutralMode(NeutralMode.Brake);
        rightmotor2.setNeutralMode(NeutralMode.Brake);


        leftmotor2.follow(leftmotor);
        rightmotor2.follow(rightmotor);

        leftmotor.setInverted(true);
        leftmotor2.setInverted(true);

    }

    @Override
    public void periodic() {
        // Powers updated in RobotContainer periodic, sent to motors here
        // Divided so that the motor powers will never exceed MAXSPEED (or negative
        // MAXSPEED)
        // (Basically just scales the motors when turning)

        // System.out.println(forwardpower);
        scaler = (Math.max(1., (Math.max(Math.abs(forwardpower - turnpower), Math.abs(forwardpower + turnpower)))));

        leftmotor.set((forwardpower - turnpower) / scaler);
        rightmotor.set((forwardpower + turnpower) / scaler);
    }
}
