package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A single joystick on port 0, with rotation bound to the Z-rotate axis. Swerve relative is bound
 * to the trigger, balancing is bound to the middle-right button, and field reset is bound to the 
 * middle button.
 */
public class TwistJoystickDriveController extends BaseDriveController {
    private final Joystick joystick = new Joystick(0);
    private final JoystickButton
        leftTrigger = new JoystickButton(joystick, 1),
        leftMiddleButton = new JoystickButton(joystick, 2),
        leftTopLeftButton = new JoystickButton(joystick, 3),
        leftTopRightButton = new JoystickButton(joystick, 4),
        leftMiddleLeftButton = new JoystickButton(joystick, 5),
        leftMiddleRightButton = new JoystickButton(joystick, 6),
        leftBackButton = new JoystickButton(joystick, 7);

    private final double JOYSTICK_DEADBAND = 0.08;

    @Override
    public double getForwardPower() {
        return MathUtil.applyDeadband(-joystick.getY(), JOYSTICK_DEADBAND);
    }

    @Override
    public double getLeftPower() {
        return MathUtil.applyDeadband(-joystick.getX(), JOYSTICK_DEADBAND);
    }

    @Override
    public double getRotatePower() {
        // Z-rotate
        return MathUtil.applyDeadband(-joystick.getRawAxis(3), JOYSTICK_DEADBAND);
    }

    @Override
    public boolean getSwerveRelative() {
        return joystick.getTrigger();
    }

    @Override
    public JoystickButton getBalancerButton() {
        return leftMiddleRightButton;
    }

    @Override
    public JoystickButton getflipConeButton() {
        return leftMiddleLeftButton;
    }

    @Override
    public JoystickButton getFieldResetButton() {
        return leftMiddleButton;
    }

    @Override
    public JoystickButton getCameraSwitchButton() {
        return leftTopRightButton;
    }

    @Override
    public JoystickButton getChargingStationLockButton() {
        return leftTopLeftButton;
    }
}
