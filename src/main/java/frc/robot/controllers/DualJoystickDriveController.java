package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A two-joystick drive controller on ports 0 and 1.
 */
public class DualJoystickDriveController extends BaseDriveController {
    private final Joystick leftJoystick = new Joystick(0);
    private final JoystickButton
        leftTrigger = new JoystickButton(leftJoystick, 1),
        leftMiddleButton = new JoystickButton(leftJoystick, 2),
        leftTopLeftButton = new JoystickButton(leftJoystick, 3),
        leftTopRightButton = new JoystickButton(leftJoystick, 4),
        leftMiddleLeftButton = new JoystickButton(leftJoystick, 5),
        leftMiddleRightButton = new JoystickButton(leftJoystick, 6),
        leftBackButton = new JoystickButton(leftJoystick, 7);

    private final Joystick rightJoystick = new Joystick(1);
    private final JoystickButton
        rightTrigger = new JoystickButton(rightJoystick, 1),
        rightMiddleButton = new JoystickButton(rightJoystick, 2),
        rightTopLeftButton = new JoystickButton(rightJoystick, 3),
        rightTopRightButton = new JoystickButton(rightJoystick, 4),
        rightMiddleLeftButton = new JoystickButton(rightJoystick, 5),
        rightMiddleRightButton = new JoystickButton(rightJoystick, 6),
        rightBackButton = new JoystickButton(rightJoystick, 7);

    private static final double JOYSTICK_DEADBAND = 0.08;

    @Override
    public double getForwardPower() {
        double scale = getDriveScaling();
        return MathUtil.applyDeadband(-leftJoystick.getY() * scale, JOYSTICK_DEADBAND);
    }

    @Override
    public double getLeftPower() {
        double scale = getDriveScaling();
        return MathUtil.applyDeadband(-leftJoystick.getX() * scale, JOYSTICK_DEADBAND);
    }

    @Override
    public double getRotatePower() {
        return MathUtil.applyDeadband(-rightJoystick.getX() * getTurnScaling(), JOYSTICK_DEADBAND);
    }

    @Override
    public boolean getSwerveRelative() {
        return rightJoystick.getTrigger();
    }

    @Override
    public boolean getSwerveHeadingLock() {
        return leftMiddleButton.getAsBoolean();
    }

    @Override
    public JoystickButton getBalancerButton() {
        return rightTopRightButton;
    }

    @Override
    public JoystickButton getFieldResetButton() {
        return leftTopRightButton;
    }

    @Override
    public JoystickButton getCameraSwitchButton() {
        return leftTopRightButton;
    }

    @Override
    public JoystickButton getChargingStationLockButton() {
        return leftTopLeftButton;
    }

    @Override
    public JoystickButton getAlignToClosestButton() {
        return rightMiddleButton;
    }

    @Override
    public JoystickButton getAlignLeftButton() {
        return rightMiddleLeftButton;
    }

    @Override
    public JoystickButton getAlignRightButton() {
        return rightMiddleRightButton;
    }

    @Override
    public JoystickButton getDriveForwardButton() {
        return rightTopLeftButton;
    }

    @Override
    public JoystickButton getCancelAutoAlignButton() {
        return rightBackButton;
    }

    @Override
    public boolean approachShelf() {
        return leftBackButton.getAsBoolean();
    }

    /**
     * Gets the amount to scale translational input by.
     * @return The scale to apply to translational input.
     */
    private double getDriveScaling() {
        boolean isSlowMode = leftJoystick.getTrigger();
        return isSlowMode ? 0.3 : 1.0;
    }

    /**
     * Gets the amount to scale rotational input by.
     * @return The scale to apply to rotational input.
     */
    private double getTurnScaling() {
        boolean isSlowMode = leftJoystick.getTrigger();
        return isSlowMode ? 0.25 : 0.5;
    }
}
