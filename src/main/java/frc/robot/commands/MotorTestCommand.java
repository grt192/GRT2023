package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.mover.TiltedElevatorCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public class MotorTestCommand extends SequentialCommandGroup {
    private static final double TEST_DELAY_SECS = 1;
    private static final double MECH_DELAY_SECS = 2;

    public MotorTestCommand(BaseSwerveSubsystem swerveSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem, RollerSubsystem rollerSubsystem) {
        addRequirements(swerveSubsystem, tiltedElevatorSubsystem, rollerSubsystem);
        
        addCommands(
            // Run tilted elevator to all heights
            // GROUND - CHUTE - SUBSTATION - CUBEMID - CUBEHIGH - CONEMID - CONEHIGH - SUBSTATION
            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.GROUND),
            new WaitCommand(TEST_DELAY_SECS),

            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.CHUTE),
            new WaitCommand(TEST_DELAY_SECS),

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

            new TiltedElevatorCommand(tiltedElevatorSubsystem, ElevatorState.SUBSTATION),
            new WaitCommand(MECH_DELAY_SECS),

            // TODO: run rollers in and out

            // Open motor
            new InstantCommand(() -> rollerSubsystem.openMotor()),
            new WaitCommand(MECH_DELAY_SECS),

            // Test swerve with S-curves
            FollowPathCommand.from(
                swerveSubsystem, 
                new Pose2d(), 
                List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1) //,
                    // new Translation2d(3, 0),
                    // new Translation2d(2, -1),
                    // new Translation2d(1, 1)
                ),
                new Pose2d(3, 0, new Rotation2d())
                // new Pose2d()
            ),
            FollowPathCommand.from(
                swerveSubsystem,
                new Pose2d(3, 0, new Rotation2d()),
                List.of(
                    new Translation2d(2, -1),
                    new Translation2d(1, 1)
                ),
                new Pose2d()
            )
        );
    }
}
