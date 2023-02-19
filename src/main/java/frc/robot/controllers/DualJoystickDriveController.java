package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A two-joystick drive controller on ports 0 and 1. Swerve relative is bound to the right trigger,
 * balancing is bound to the left trigger, and field reset is bound to the left middle button.
 */
public class DualJoystickDriveController extends BaseDriveController {
    private final Joystick leftJoystick = new Joystick(0);
    private final JoystickButton
        leftTrigger = new JoystickButton(leftJoystick, 1),
        leftMButton = new JoystickButton(leftJoystick, 2),
        leftTopCenterButton = new JoystickButton(leftJoystick, 3);

    private final Joystick rightJoystick = new Joystick(1);
    private final JoystickButton
        rightTrigger = new JoystickButton(rightJoystick, 1);

    private final double JOYSTICK_DEADBAND = 0.08;

    @Override
    public double getForwardPower() {
        return MathUtil.applyDeadband(-leftJoystick.getY(), JOYSTICK_DEADBAND);
    }

    @Override
    public double getLeftPower() {
        return MathUtil.applyDeadband(-leftJoystick.getX(), JOYSTICK_DEADBAND);
    }

    @Override
    public double getRotatePower() {
        return MathUtil.applyDeadband(-rightJoystick.getX(), JOYSTICK_DEADBAND);
    }

    @Override
    public boolean getSwerveRelative() {
        return rightJoystick.getTrigger();
    }

    @Override
    public JoystickButton getBalancerButton() {
        return leftTrigger;
    }

    @Override
    public JoystickButton getFieldResetButton() {
        return leftMButton;
    }

    @Override
    public JoystickButton getCameraSwitchButton() {
        return leftTopCenterButton;
    }
}
