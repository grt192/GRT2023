package frc.robot.commands.dropping;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.commands.swerve.GoToPointCommand;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class AlignToNodeCommand extends ParallelCommandGroup {
    private static final double ELEVATOR_TOLERANCE_RADS = Math.toRadians(10.0);

    /**
     * Creates an align-to-node command from a given swerve subsystem, elevator subsystem, target place position, and
     * whether the robot is currently on the red team.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param placePosition The target place position to align with.
     * @param isRed Whether the robot is on the red team.
     */
    public AlignToNodeCommand(
        BaseSwerveSubsystem swerveSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        PlacePosition targetPlacePosition, boolean isRed
    ) {
        GoToPointCommand driveCommand = new GoToPointCommand(swerveSubsystem, targetPlacePosition.alignPosition.getPose(isRed));

        addCommands(
            driveCommand,
            new WaitUntilCommand(() -> driveCommand.isHeadingAligned(ELEVATOR_TOLERANCE_RADS)).andThen(
                new InstantCommand(() -> tiltedElevatorSubsystem.setState(targetPlacePosition.elevatorState), tiltedElevatorSubsystem)
            )
        );
    }
}
