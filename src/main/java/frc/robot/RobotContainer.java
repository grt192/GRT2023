// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BalancerCommand;
import frc.robot.jetson.JetsonConnection;
import frc.robot.subsystems.Tank;
import frc.robot.subsystems.swerve.MissileShellSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem2020;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final BaseSwerveSubsystem swerveSubsystem;
    
    private final Tank tank = new Tank();
    private final AHRS ahrs;


    private final JetsonConnection jetsonConnection;

    // Controllers and buttons
    private final Joystick joystick = new Joystick(2);
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
    private final Command balanceCommand;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerveSubsystem = new SwerveSubsystem2020();
        ahrs = new AHRS(SPI.Port.kMXP);
        balanceCommand = new BalancerCommand(swerveSubsystem,ahrs); // pass DT of choice into balancer

        jetsonConnection = new JetsonConnection();
        jetsonConnection.start();

        // Configure the button bindings
        configureButtonBindings();

        // Add auton sequences to the chooser and add the chooser to shuffleboard
        // TODO: shuffleboard

        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("Skip auton", new InstantCommand());
        driveRBumper.whileTrue(balanceCommand);
        
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
         /* 
        swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
            double xPower = -driveController.getLeftY();
            double yPower = -driveController.getLeftX();
            double angularPower = -driveController.getRightX();
            swerveSubsystem.setSwerveDrivePowers(xPower, yPower, angularPower);
        }, swerveSubsystem));

        driveAButton.onTrue(new InstantCommand(swerveSubsystem::resetFieldAngle, swerveSubsystem));
        */
    }


    void periodic(){
        if(!driveRBumper.getAsBoolean()){
            tank.forwardComponent = -0.75 * driveController.getLeftY(); // 1 is forward (adjusted from -1 forward)
            tank.sideComponent = 0.75 * driveController.getRightX(); // 1 is right
        }
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
