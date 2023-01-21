package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public abstract class BaseDriveController {
    /**
     * Gets the forward power commanded by the controller.
     * @return The [-1.0, 1.0] forward power.
     */
    public abstract double getForward();

    /**
     * Gets the left power commanded by the controller.
     * @return The [-1.0, 1.0] left power.
     */
    public abstract double getLeft();

    /**
     * Gets the rotational power commanded by the controller.
     * @return The [-1.0, 1.0] angular power.
     */
    public abstract double getRotate();

    /**
     * Gets the whether swerve should be run in robot-relative mode.
     * @return Whether swerve should be run in robot-relative mode.
     */
    public abstract boolean getSwerveRelative();

    public abstract JoystickButton getBalancerButton();
    public abstract JoystickButton getFieldResetButton();
}
