package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A single XboxController on port 0. Swerve relative is bound to the right trigger, balancing is bound to the
 * right bumper, and field reset is bound to the A button.
 */
public class XboxDriveController extends BaseDriveController {
    private final XboxController driveController = new XboxController(0);
    private final JoystickButton 
        driveAButton = new JoystickButton(driveController, XboxController.Button.kA.value),
        driveBButton = new JoystickButton(driveController, XboxController.Button.kB.value),
        driveXButton = new JoystickButton(driveController, XboxController.Button.kX.value),
        driveYButton = new JoystickButton(driveController, XboxController.Button.kY.value),
        driveLBumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value),
        driveRBumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);

    @Override
    public double getForwardPower() {
        return -driveController.getLeftY();
    }

    @Override
    public double getLeftPower() {
        return -driveController.getLeftX();
    }

    @Override
    public double getRotatePower() {
        return -driveController.getRightX();
    }

    @Override
    public boolean getSwerveRelative() {
        return driveController.getRightTriggerAxis() > 0.75;
    }

    @Override
    public boolean getHeadingLock() {
        return driveController.getLeftTriggerAxis() > 0.75;
    }

    @Override
    public JoystickButton getBalancerButton() {
        return driveRBumper;
    }

    @Override
    public JoystickButton getFieldResetButton() {
        return driveAButton;
    }

    @Override
    public JoystickButton getCameraSwitchButton() {
        return driveYButton;
    }

    @Override
    public JoystickButton getChargingStationLockButton() {
        return driveBButton;
    }
}
