// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.BalancerCommand;
import frc.robot.commands.sequences.BlueBalanceAuton;
import frc.robot.commands.sequences.BlueBottomAuton;
import frc.robot.commands.sequences.BlueTopAuton;
import frc.robot.commands.sequences.RedBalanceAuton;
import frc.robot.commands.sequences.RedBottomAuton;
import frc.robot.commands.sequences.RedTopAuton;
import frc.robot.commands.sequences.test.BoxAutonSequence;
import frc.robot.commands.sequences.test.GRTAutonSequence;
import frc.robot.commands.sequences.test.HighRotationLinePath;
import frc.robot.commands.sequences.test.RotatingSCurveAutonSequence;
import frc.robot.commands.sequences.test.StraightLinePath;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.TwistJoystickDriveController;
import frc.robot.controllers.XboxDriveController;
import frc.robot.vision.PhotonWrapper;
import frc.robot.subsystems.drivetrain.TankSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.drivetrain.MissileShellSwerveSubsystem;
import frc.robot.subsystems.drivetrain.MissileShellSwerveSweeperSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem2020;
import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;

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
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    private final Superstructure superstructure;
    private final PhotonWrapper photonWrapper;

    // Controllers and buttons
    private final BaseDriveController driveController;

    private UsbCamera front;
    private UsbCamera back;
    SimpleWidget cameraSelection;
    private int CameraID = 0;
    VideoSink server;

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

    private final XboxController mechController = new XboxController(2);
    private final JoystickButton
        mechAButton = new JoystickButton(mechController, XboxController.Button.kA.value),
        mechBButton = new JoystickButton(mechController, XboxController.Button.kB.value),
        mechXButton = new JoystickButton(mechController, XboxController.Button.kX.value),
        mechYButton = new JoystickButton(mechController, XboxController.Button.kY.value),
        mechLBumper = new JoystickButton(mechController, XboxController.Button.kLeftBumper.value),
        mechRBumper = new JoystickButton(mechController, XboxController.Button.kRightBumper.value);

        

    // Commands
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
    private final SendableChooser<Command> autonChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveController = new XboxDriveController();

        photonWrapper = new PhotonWrapper();
        front =  CameraServer.startAutomaticCapture(0);
        back =  CameraServer.startAutomaticCapture(1);
        
        server = CameraServer.getServer();

        driveSubsystem = new SwerveSubsystem(photonWrapper);
        rollerSubsystem = new RollerSubsystem();
        tiltedElevatorSubsystem = new TiltedElevatorSubsystem();

        superstructure = new Superstructure(rollerSubsystem, tiltedElevatorSubsystem);
        cameraSelection = Shuffleboard.getTab("Driver Camera").add("Intake Camera", server);

        // Configure the button bindings
        configureButtonBindings();

        // Add auton sequences to the chooser and add the chooser to shuffleboard
        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("Skip auton", new InstantCommand());

        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            final BaseSwerveSubsystem swerveSubsystem = (BaseSwerveSubsystem) driveSubsystem;

            autonChooser.addOption("Straight-line path", new StraightLinePath(swerveSubsystem));
            autonChooser.addOption("High rotation straight-line path", new HighRotationLinePath(swerveSubsystem));
            autonChooser.addOption("Rotating S-curve", new RotatingSCurveAutonSequence(swerveSubsystem));
            autonChooser.addOption("Box auton", new BoxAutonSequence(swerveSubsystem));
            autonChooser.addOption("GRT path", new GRTAutonSequence(swerveSubsystem));

            autonChooser.addOption("Red top auton", new RedTopAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem));
            autonChooser.addOption("Red balance auton", new RedBalanceAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem));
            autonChooser.addOption("Red bottom auton", new RedBottomAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem));

            autonChooser.addOption("Blue top auton", new BlueTopAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem));
            // autonChooser.addOption("Blue balance auton", new BlueBalanceAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem));
            autonChooser.addOption("Blue bottom auton", new BlueBottomAuton(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem));
        }

        shuffleboardTab.add(autonChooser)
            .withPosition(8, 0)
            .withSize(4, 2);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driveController.getCameraSwitchButton()
        .onTrue(new InstantCommand(() -> {
            if(CameraID == 0){
                server.setSource(front);
                CameraID = 1;
                System.out.println("CAMERA SET TO FRONT");
            }
            else{
                server.setSource(back);
                CameraID = 0;
                System.out.println("CAMERA SET TO BACK");
            }
        }));
        
        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            final BaseSwerveSubsystem swerveSubsystem = (BaseSwerveSubsystem) driveSubsystem;

            swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
                double xPower = driveController.getForwardPower();
                double yPower = driveController.getLeftPower();
                double angularPower = driveController.getRotatePower();
                boolean relative = driveController.getSwerveRelative();
                swerveSubsystem.setDrivePowers(xPower, yPower, angularPower, relative);
            }, swerveSubsystem));

            driveController.getFieldResetButton().onTrue(new InstantCommand(swerveSubsystem::resetFieldAngle, swerveSubsystem));
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
