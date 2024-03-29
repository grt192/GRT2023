// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;
    private Command testCommand;

    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        if (robotContainer.driveSubsystem instanceof BaseSwerveSubsystem) {
            BaseSwerveSubsystem swerveSubsystem = (BaseSwerveSubsystem) robotContainer.driveSubsystem;
            swerveSubsystem.setVisionEnabled(false);
        }

        // Schedule the autonomous command and cancel testing
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (testCommand != null) testCommand.cancel();
        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (robotContainer.driveSubsystem instanceof BaseSwerveSubsystem) {
            BaseSwerveSubsystem swerveSubsystem = (BaseSwerveSubsystem) robotContainer.driveSubsystem;
            swerveSubsystem.setVisionEnabled(true);
            swerveSubsystem.setChargingStationLocked(false);
        }

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
        if (testCommand != null) testCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Schedule the test command and cancel auton
        testCommand = robotContainer.getTestCommand();
        if (autonomousCommand != null) autonomousCommand.cancel();
        if (testCommand != null) testCommand.schedule();
    }

    @Override
    public void testPeriodic() {}
}
