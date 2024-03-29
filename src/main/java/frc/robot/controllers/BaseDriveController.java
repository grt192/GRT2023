package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public abstract class BaseDriveController {
    /**
     * Gets the forward power commanded by the controller.
     * @return The [-1.0, 1.0] forward power.
     */
    public abstract double getForwardPower();

    /**
     * Gets the left power commanded by the controller.
     * @return The [-1.0, 1.0] left power.
     */
    public abstract double getLeftPower();

    /**
     * Gets the rotational power commanded by the controller.
     * @return The [-1.0, 1.0] angular power.
     */
    public abstract double getRotatePower();

    /**
     * Gets whether swerve should be run in robot-relative mode.
     * @return Whether swerve should be run in robot-relative mode.
     */
    public abstract boolean getSwerveRelative();

    /**
     * Gets whether swerve should heading-lock to aim towards the grid.
     * @return Whether swerve should heading-lock towards the grid.
     */
    public abstract boolean getSwerveHeadingLock();

    /**
     * Gets whether the swerve is approaching the shelf to intake a piece.
     * @return Whether the swerve should automatically approach the shelf.
     */
    public boolean approachShelf() {
        return false;
    }

    public abstract JoystickButton getBalancerButton();
    public abstract JoystickButton getFieldResetButton();
    public abstract JoystickButton getCameraSwitchButton();
    public abstract JoystickButton getChargingStationLockButton();

    public abstract JoystickButton getAlignToClosestButton();
    public abstract JoystickButton getAlignLeftButton();
    public abstract JoystickButton getAlignRightButton();
    public abstract JoystickButton getDriveForwardButton();
    public abstract JoystickButton getCancelAutoAlignButton();
}
