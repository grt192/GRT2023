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
import static frc.robot.Constants.CarriageConstants.*;
import static frc.robot.Constants.TankConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import frc.robot.subsystems.Tank;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Carriage;


import frc.robot.shuffleboard.GRTShuffleboardTab;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Tank tank = new Tank();
    private final Intake intake = new Intake();
    private final Carriage carriage = new Carriage();

    // Controllers and buttons
    public final XboxController controller = new XboxController(0);
    

    // Commands
    private final SendableChooser<Command> autonChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
         

        // Configure the button bindings
        configureButtonBindings();

        // Add auton sequences to the chooser and add the chooser to shuffleboard
        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("Skip auton", new InstantCommand());

        new GRTShuffleboardTab("Drivetrain").addWidget("Auton sequence", autonChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    void periodic(){
        tank.forwardComponent = -1 * controller.getLeftY(); // 1 is forward (adjusted from -1 forward)
        tank.sideComponent = controller.getRightX(); // 1 is right

        if(controller.getBButton()){ //engage the intake arm by pressing B on the controller
            intake.intake_down = true;

        }
        if(controller.getXButton()){
            intake.intake_on = !intake.intake_on; //toggle on/off intake
        }

        if(controller.getRightBumper()){
        carriage.liftCarriage = !carriage.liftCarriage; // carriage lift toggle switch (true --> false or false --> true)
        }

        if(controller.getLeftBumper()){ //open the door toggle for carriage (left bumper)
            carriage.openDoor = !carriage.openDoor;
        }

    }
}
