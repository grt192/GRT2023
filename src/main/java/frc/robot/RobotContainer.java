// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.BalancerCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.TwistJoystickDriveController;
import frc.robot.controllers.XboxDriveController;
import frc.robot.jetson.JetsonConnection;
import frc.robot.subsystems.drivetrain.TankSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.drivetrain.MissileShellSwerveSubsystem;
import frc.robot.subsystems.drivetrain.MissileShellSwerveSweeperSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
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
    // private final GripperSubsytem gripperSubsystem;
    // private final RollerSubsystem rollerSubsystem;

    // private final JetsonConnection jetsonConnection;

    // Controllers and buttons
    private final BaseDriveController driveController;

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

    private final XboxController mechController = new XboxController(5);
    private final JoystickButton 
        mechAButton = new JoystickButton(mechController, XboxController.Button.kA.value),
        mechBButton = new JoystickButton(mechController, XboxController.Button.kB.value),
        mechXButton = new JoystickButton(mechController, XboxController.Button.kX.value),
        mechYButton = new JoystickButton(mechController, XboxController.Button.kY.value),
        mechLBumper = new JoystickButton(mechController, XboxController.Button.kLeftBumper.value),
        mechRBumper = new JoystickButton(mechController, XboxController.Button.kRightBumper.value);

    // Commands
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Auton");
    private final SendableChooser<Command> autonChooser;
    private final BalancerCommand balancerCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveController = new XboxDriveController();

        driveSubsystem = new SwerveSubsystem();
        // gripperSubsystem = new GripperSubsytem();
        // rollerSubsystem = new RollerSubsystem();

        balancerCommand = new BalancerCommand(driveSubsystem);

        // jetsonConnection = new JetsonConnection();
        // jetsonConnection.start();

        // Configure the button bindings
        configureButtonBindings();

        // Add auton sequences to the chooser and add the chooser to shuffleboard
        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("Skip auton", new InstantCommand());

        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            final BaseSwerveSubsystem swerveSubsystem = (BaseSwerveSubsystem) driveSubsystem;

            // S-curve auton
            autonChooser.addOption("Rotating S-curve", new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(), 
                List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
                ),
                new Pose2d(3, 0, Rotation2d.fromDegrees(90))
            ).andThen(new FollowPathCommand(
                swerveSubsystem,
                new Pose2d(3, 0, Rotation2d.fromDegrees(90)),
                List.of(
                    new Translation2d(2, -1),
                    new Translation2d(1, 1)
                ),
                new Pose2d(),
                true
            )));

            // Box auton
            autonChooser.addOption("Box auton", new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(),
                List.of(), 
                new Pose2d(2, 0, new Rotation2d())
            ).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(2, 0, new Rotation2d()), 
                List.of(), 
                new Pose2d(2, -1, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(2, -1, new Rotation2d()), 
                List.of(), 
                new Pose2d(3, -1, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(3, -1, new Rotation2d()), 
                List.of(), 
                new Pose2d(3, 1, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(3, 1, new Rotation2d()), 
                List.of(), 
                new Pose2d(2, 1, new Rotation2d()),
                true
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(2, 1, new Rotation2d()), 
                List.of(), 
                new Pose2d(2, 0, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(2, 0, new Rotation2d()), 
                List.of(), 
                new Pose2d(),
                true
            )));

            // "GRT" auton
            autonChooser.addOption("GRT path", new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(),
                List.of(),
                new Pose2d(1, 1, new Rotation2d())
            ).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(1, 1, new Rotation2d()),
                List.of(),
                new Pose2d(),
                true
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(),
                List.of(
                    new Translation2d(0.5, -1)
                ),
                new Pose2d(1, 0, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(1, 0, new Rotation2d()),
                List.of(),
                new Pose2d(0.5, 0, new Rotation2d()),
                true
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(0.5, 0, new Rotation2d()),
                List.of(),
                new Pose2d(1.5, -1, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(1.5, -1, new Rotation2d()),
                List.of(),
                new Pose2d(1.5, 1, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(1.5, 1, new Rotation2d()),
                List.of(
                    new Translation2d(2.5, 0.5)
                ),
                new Pose2d(1.5, 0, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(1.5, 0, new Rotation2d()),
                List.of(),
                new Pose2d(2.5, -1, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(2.5, -1, new Rotation2d()),
                List.of(),
                new Pose2d(4, -1, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(4, -1, new Rotation2d()),
                List.of(),
                new Pose2d(4, 1, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(4, 1, new Rotation2d()),
                List.of(),
                new Pose2d(4.5, 1, new Rotation2d())
            )).andThen(new FollowPathCommand(
                swerveSubsystem, 
                new Pose2d(4.5, 1, new Rotation2d()),
                List.of(),
                new Pose2d(3.5, 1, new Rotation2d()),
                true
            )));
        }

        shuffleboardTab.add(autonChooser)
            .withPosition(0, 0)
            .withSize(4, 2);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driveController.getBalancerButton().whileTrue(balancerCommand);

        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            final BaseSwerveSubsystem swerveSubsystem = (BaseSwerveSubsystem) driveSubsystem;

            swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
                double xPower = driveController.getForward();
                double yPower = driveController.getLeft();
                double angularPower = driveController.getRotate();
                boolean relative = driveController.getSwerveRelative();
                swerveSubsystem.setDrivePowers(xPower, yPower, angularPower, relative);
            }, swerveSubsystem));

            driveController.getFieldResetButton().onTrue(new InstantCommand(swerveSubsystem::resetFieldAngle, swerveSubsystem));
        } else if (driveSubsystem instanceof TankSubsystem) {
            final TankSubsystem tankSubsystem = (TankSubsystem) driveSubsystem;

            tankSubsystem.setDefaultCommand(new RunCommand(() -> {
                double forwardPower = 0.75 * driveController.getForward();
                double turnPower = 0.75 * driveController.getRotate();
                tankSubsystem.setDrivePowers(forwardPower, turnPower);
            }, tankSubsystem));
        } else if (driveSubsystem instanceof MissileShellSwerveSubsystem) {
            final MissileShellSwerveSubsystem swerveSubsystem = (MissileShellSwerveSubsystem) driveSubsystem;

            swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
                double xPower = driveController.getForward();
                double yPower = driveController.getLeft();
                swerveSubsystem.setDrivePowers(xPower, yPower);
            }, swerveSubsystem));
        }

        /*
        rollerSubsystem.setDefaultCommand(new RunCommand(() -> {
            double rollPower = mechController.getRightTriggerAxis() - mechController.getLeftTriggerAxis();
            rollerSubsystem.setRollPower(rollPower);
        }, rollerSubsystem));

        mechAButton.onTrue(new InstantCommand(gripperSubsystem::gripToggle, gripperSubsystem));
        */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
