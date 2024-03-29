package frc.robot.commands.pretest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.dropping.DropperChooserCommand;
import frc.robot.commands.grabber.RollerIntakeCommand;
import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class MotorTestCommand extends SequentialCommandGroup {
    private static final double TEST_DELAY_SECS = 1;
    private static final double MECH_DELAY_SECS = 2;

    private static final double SWERVE_DRIVE_POWER = 0.5;
    private static final double SWERVE_DRIVE_TIME_SECS = 3;

    public MotorTestCommand(BaseSwerveSubsystem swerveSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, RollerSubsystem rollerSubsystem) {
        addRequirements(swerveSubsystem, tiltedElevatorSubsystem, rollerSubsystem);

        addCommands(
            // Lock swerve
            new InstantCommand(swerveSubsystem::applyLock, swerveSubsystem),

            // Run tilted elevator to all heights
            // GROUND - CHUTE - INTAKE CONE - SUBSTATION - CUBEMID - CUBEHIGH - CONEMID - CONEHIGH - CONEMID 
            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.GROUND),
            new WaitCommand(TEST_DELAY_SECS),

            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.CHUTE),
            new WaitCommand(TEST_DELAY_SECS),

            new RollerIntakeCommand(rollerSubsystem),
            new WaitCommand(4), // to not take any fingers off

            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.SUBSTATION),
            new WaitCommand(TEST_DELAY_SECS),

            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.CUBE_MID),
            new WaitCommand(TEST_DELAY_SECS),

            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.CUBE_HIGH),
            new WaitCommand(TEST_DELAY_SECS),

            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.CONE_MID),
            new WaitCommand(TEST_DELAY_SECS),

            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.CONE_HIGH),
            new WaitCommand(TEST_DELAY_SECS),

            // return to mid before dropping
            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.CONE_MID),
            new WaitCommand(TEST_DELAY_SECS),

            // Test dropper command from `CONE_MID`
            DropperChooserCommand.getSequence(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem),
            new WaitCommand(MECH_DELAY_SECS),

            // // Sweep rollers from [-1.0, 1.0] over 2 seconds
            // new RollerSweepCommand(rollerSubsystem, 2.0),
            // new WaitCommand(TEST_DELAY_SECS),

            // // Open motor
            // new InstantCommand(rollerSubsystem::openMotor, rollerSubsystem),
            // new WaitCommand(MECH_DELAY_SECS),

            // Test swerve with forward, back, left, and right powers, and turning counterclockwise and clockwise
            new InstantCommand(() -> swerveSubsystem.setDrivePowers(SWERVE_DRIVE_POWER, 0, 0, true)),
            new WaitCommand(SWERVE_DRIVE_TIME_SECS),

            new InstantCommand(() -> swerveSubsystem.setDrivePowers(-SWERVE_DRIVE_POWER, 0, 0, true)),
            new WaitCommand(SWERVE_DRIVE_TIME_SECS),

            new InstantCommand(() -> swerveSubsystem.setDrivePowers(0, SWERVE_DRIVE_POWER, 0, true)),
            new WaitCommand(SWERVE_DRIVE_TIME_SECS),

            new InstantCommand(() -> swerveSubsystem.setDrivePowers(0, -SWERVE_DRIVE_POWER, 0, true)),
            new WaitCommand(SWERVE_DRIVE_TIME_SECS),

            new InstantCommand(() -> swerveSubsystem.setDrivePowers(0, 0, SWERVE_DRIVE_POWER, true)),
            new WaitCommand(SWERVE_DRIVE_TIME_SECS),

            new InstantCommand(() -> swerveSubsystem.setDrivePowers(0, 0, -SWERVE_DRIVE_POWER, true)),
            new WaitCommand(SWERVE_DRIVE_TIME_SECS),

            new InstantCommand(() -> swerveSubsystem.setDrivePowers(0, 0, 0, true))
        );
    }
}
