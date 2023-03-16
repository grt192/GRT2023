package frc.robot.subsystems.roller;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.util.MotorUtil;
import static frc.robot.Constants.RollerConstants.*;

public class RollerIOReal implements RollerIO {
    private final WPI_TalonSRX leftBeak;
    private final WPI_TalonSRX rightBeak;
    private final WPI_TalonSRX openMotor;

    private final DigitalInput limitSwitch;
    private final ColorSensorV3 crolorSensor;

    public RollerIOReal() {
        leftBeak = MotorUtil.createTalonSRX(LEFT_ID);
        leftBeak.setInverted(true);
        // leftBeak.setInverted(false);
        leftBeak.setNeutralMode(NeutralMode.Brake);

        rightBeak = MotorUtil.createTalonSRX(RIGHT_ID);
        rightBeak.follow(leftBeak);
        rightBeak.setInverted(InvertType.OpposeMaster);
        rightBeak.setNeutralMode(NeutralMode.Brake);

        openMotor = MotorUtil.createTalonSRX(OPEN_ID);
        openMotor.setNeutralMode(NeutralMode.Brake);
        openMotor.setInverted(true);

        limitSwitch = new DigitalInput(LIMIT_SWITCH_ID);
        crolorSensor = new ColorSensorV3(I2C.Port.kMXP);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.limitSwitchPressed = !limitSwitch.get();
        inputs.proximitySensorReading = crolorSensor.getProximity();

        Color detectedColor = crolorSensor.getColor();
        inputs.colorSensorRed = detectedColor.red * 255;
        inputs.colorSensorGreen = detectedColor.green * 255;
        inputs.colorSensorBlue = detectedColor.blue * 255;
    }
}
