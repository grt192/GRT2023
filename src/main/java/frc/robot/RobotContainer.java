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
import frc.robot.subsystems.drivetrain.MissileShellSwerveSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem2020;
import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.MoverSubsystem;
import frc.robot.subsystems.MoverSubsystem.MoverPosition;
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
    private final GripperSubsytem gripperSubsystem;
    private final RollerSubsystem rollerSubsystem;

    //private final JetsonConnection jetsonConnection;

    private final MoverSubsystem moverSubsystem;

    // Controllers and buttons
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
        driveSubsystem = new TankSubsystem();
        gripperSubsystem = new GripperSubsytem();
        rollerSubsystem = new RollerSubsystem();

        balancerCommand = new BalancerCommand(driveSubsystem);

        // jetsonConnection = new JetsonConnection();
        // jetsonConnection.start();
        
        moverSubsystem = new MoverSubsystem();

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
        } else if (driveSubsystem instanceof MissileShellSwerveSubsystem) {
            final MissileShellSwerveSubsystem swerveSubsystem = (MissileShellSwerveSubsystem) driveSubsystem;

            swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
                double xPower = -driveController.getLeftY();
                double yPower = -driveController.getLeftX();
                swerveSubsystem.setDrivePowers(xPower, yPower);
            }, swerveSubsystem));
        }
        moverSubsystem.setDefaultCommand(new RunCommand(() -> {
            double xPower = mechController.getLeftX();
            double yPower = -mechController.getRightY();
            moverSubsystem.setPowers(xPower, yPower);
        }, moverSubsystem));

        mechYButton.onTrue(new InstantCommand(() ->{
            if(moverSubsystem.getState() == MoverPosition.GROUND){
                moverSubsystem.setState(MoverPosition.SUBSTATION);
            } else {
                moverSubsystem.setState(MoverPosition.GROUND);
            }
        }, moverSubsystem));

        mechRBumper.onTrue(new InstantCommand(() ->{
            if(moverSubsystem.getState() == MoverPosition.CUBEHIGH){
                moverSubsystem.setState(MoverPosition.CUBEMID);
            } else {
                moverSubsystem.setState(MoverPosition.CUBEHIGH);
            }
        }, moverSubsystem));

        mechLBumper.onTrue(new InstantCommand(() ->{
            if(moverSubsystem.getState() == MoverPosition.CONEHIGH){
                moverSubsystem.setState(MoverPosition.CONEMID);
            } else {
                moverSubsystem.setState(MoverPosition.CONEHIGH);
            }
        }, moverSubsystem));

        rollerSubsystem.setDefaultCommand(new RunCommand(() -> {
            double rollPower = mechController.getRightTriggerAxis() - mechController.getLeftTriggerAxis();
            rollerSubsystem.setRollPower(rollPower);
        }, rollerSubsystem));

        mechAButton.onTrue(new InstantCommand(gripperSubsystem::gripToggle, gripperSubsystem));

        mechXButton.onTrue(new InstantCommand(() ->{
            moverSubsystem.setState(MoverPosition.VERTICAL);
        }));
    }

    public BaseSwerveSubsystem getSwerveSubsystem(){
        if (driveSubsystem instanceof BaseSwerveSubsystem){
            return (BaseSwerveSubsystem) driveSubsystem;
        }

        else{
            throw new RuntimeException("no swerveSubsystem");
        }
    }

    public GripperSubsytem getGripperSubsytem(){
        return gripperSubsystem;
    }

    public MoverSubsystem getMoverSubsystem(){
        return moverSubsystem;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
