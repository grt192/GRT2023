// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.BalancerCommand;
import frc.robot.jetson.JetsonConnection;
import frc.robot.subsystems.drivetrain.TankSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem2020;
import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.GripperSubsytem;
import frc.robot.subsystems.RollerSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final BaseDrivetrain driveSubsystem;

    //private final JetsonConnection jetsonConnection;

    private final GripperSubsytem gripper;

    private final RollerSubsystem roller;

    // Controllers and buttons
    // private final Joystick joystick = new Joystick(2);
    private final GenericHID switchboard = new GenericHID(3);
    private final JoystickButton
        tlSwitch = new JoystickButton(switchboard, 3),
        tmSwitch = new JoystickButton(switchboard, 2),
        trSwitch = new JoystickButton(switchboard, 1),
        mlSwitch = new JoystickButton(switchboard, 6),
        mmSwitch = new JoystickButton(switchboard, 5),
        mrSwitch = new JoystickButton(switchboard, 4),
        blSwitch = new JoystickButton(switchboard, 9),
        bmSwitch = new JoystickButton(switchboard, 8),
        brSwitch = new JoystickButton(switchboard, 7);

    private final XboxController driveController = new XboxController(0);
    private final JoystickButton 
        driveAButton = new JoystickButton(driveController, XboxController.Button.kA.value),
        driveBButton = new JoystickButton(driveController, XboxController.Button.kB.value),
        driveXButton = new JoystickButton(driveController, XboxController.Button.kX.value),
        driveYButton = new JoystickButton(driveController, XboxController.Button.kY.value),
        driveLBumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value),
        driveRBumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);

    private final XboxController mechController = new XboxController(1);
    private final JoystickButton 
        mechAButton = new JoystickButton(mechController, XboxController.Button.kA.value),
        mechBButton = new JoystickButton(mechController, XboxController.Button.kB.value),
        mechXButton = new JoystickButton(mechController, XboxController.Button.kX.value),
        mechYButton = new JoystickButton(mechController, XboxController.Button.kY.value),
        mechLBumper = new JoystickButton(mechController, XboxController.Button.kLeftBumper.value),
        mechRBumper = new JoystickButton(mechController, XboxController.Button.kRightBumper.value);

    // Commands
    private final SendableChooser<Command> autonChooser;
    private final BalancerCommand balancerCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveSubsystem = new SwerveSubsystem2020();
        balancerCommand = new BalancerCommand(driveSubsystem);

        // jetsonConnection = new JetsonConnection();
        // jetsonConnection.start();

        gripper = new GripperSubsytem();

        roller = new RollerSubsystem();

        // Configure the button bindings
        configureButtonBindings();

        // Add auton sequences to the chooser and add the chooser to shuffleboard
        // TODO: shuffleboard

        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("Skip auton", new InstantCommand());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driveRBumper.whileTrue(balancerCommand);

        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            final BaseSwerveSubsystem swerveSubsystem = (BaseSwerveSubsystem) driveSubsystem;

            swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
                double xPower = -driveController.getLeftY();
                double yPower = -driveController.getLeftX();
                double angularPower = -driveController.getRightX();
                boolean relative = driveController.getRightTriggerAxis() > 0.75;
                swerveSubsystem.setDrivePowers(xPower, yPower, angularPower, relative);
            }, swerveSubsystem));

            driveAButton.onTrue(new InstantCommand(swerveSubsystem::resetFieldAngle, swerveSubsystem));
        } else if (driveSubsystem instanceof TankSubsystem) {
            final TankSubsystem tankSubsystem = (TankSubsystem) driveSubsystem;

            tankSubsystem.setDefaultCommand(new RunCommand(() -> {
                double forwardPower = -0.75 * driveController.getLeftY();
                double turnPower = 0.75 * driveController.getRightX();
                tankSubsystem.setDrivePowers(forwardPower, turnPower);
            }, tankSubsystem));
        }
        
        mechAButton.onTrue(new InstantCommand(gripper::gripToggle, gripper));
        mechBButton.onTrue(new InstantCommand(roller::rollToggle, roller));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

}
