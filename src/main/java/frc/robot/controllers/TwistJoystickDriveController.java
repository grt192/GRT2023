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
        leftMButton = new JoystickButton(joystick, 2),
        leftMRButton = new JoystickButton(joystick, 6);

    private final double JOYSTICK_DEADBAND = 0.08;

    @Override
    public double getForward() {
        return MathUtil.applyDeadband(-joystick.getY(), JOYSTICK_DEADBAND);
    }

    @Override
    public double getLeft() {
        return MathUtil.applyDeadband(-joystick.getX(), JOYSTICK_DEADBAND);
    }

    @Override
    public double getRotate() {
        // Z-rotate
        return MathUtil.applyDeadband(-joystick.getRawAxis(3), JOYSTICK_DEADBAND);
    }

    @Override
    public boolean getSwerveRelative() {
        return joystick.getTrigger();
    }

    @Override
    public JoystickButton getBalancerButton() {
        return leftMRButton;
    }

    @Override
    public JoystickButton getFieldResetButton() {
        return leftMButton;
    }
}
