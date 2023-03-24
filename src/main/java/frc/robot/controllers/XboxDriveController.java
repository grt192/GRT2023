package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A single Xbox controller on port 0.
 */
public class XboxDriveController extends BaseDriveController {
    private final XboxController driveController = new XboxController(0);
    private final JoystickButton 
        driveAButton = new JoystickButton(driveController, XboxController.Button.kA.value),
        driveBButton = new JoystickButton(driveController, XboxController.Button.kB.value),
        driveXButton = new JoystickButton(driveController, XboxController.Button.kX.value),
        driveYButton = new JoystickButton(driveController, XboxController.Button.kY.value),
        driveLBumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value),
        driveRBumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value),
        driveLStickButton = new JoystickButton(driveController, XboxController.Button.kLeftStick.value),
        driveRStickButton = new JoystickButton(driveController, XboxController.Button.kRightStick.value),
        driveBackButton = new JoystickButton(driveController, XboxController.Button.kBack.value),
        driveStartButton = new JoystickButton(driveController, XboxController.Button.kStart.value);

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
    public boolean getSwerveHeadingLock() {
        return driveController.getLeftTriggerAxis() > 0.75;
    }

    @Override
    public JoystickButton getBalancerButton() {
        return driveLStickButton;
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

    @Override
    public JoystickButton getAlignToClosestButton() {
        return driveRStickButton;
    }

    @Override
    public JoystickButton getAlignLeftButton() {
        return driveLBumper;
    }

    @Override
    public JoystickButton getAlignRightButton() {
        return driveRBumper;
    }

    @Override
    public JoystickButton getDriveForwardButton() {
        return driveStartButton;
    }

    @Override
    public JoystickButton getCancelAutoAlignButton() {
        return driveXButton;
    }
}
