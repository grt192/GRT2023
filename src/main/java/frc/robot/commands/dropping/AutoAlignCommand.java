package frc.robot.commands.dropping;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class AutoAlignCommand extends InstantCommand {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    private final Pose2d alignPose;
    private final ElevatorState elevatorState;

    private FollowPathCommand wrappedDriveCommand;

    /**
     * Creates an auto-align command from a given swerve subsystem, elevator subsystem, target place position, and
     * whether the robot is currently on the red team.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param placePosition The target place position to align with.
     * @param isRed Whether the robot is on the red team.
     */
    public AutoAlignCommand(
        BaseSwerveSubsystem swerveSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        PlacePosition placePosition, boolean isRed
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        this.alignPose = placePosition.alignPosition.getPose(isRed);
        this.elevatorState = placePosition.elevatorState;

        addRequirements(swerveSubsystem, tiltedElevatorSubsystem);
    }

    @Override
    public void initialize() {
        tiltedElevatorSubsystem.setState(elevatorState);

        // Schedule follow path command to align with node
        Pose2d initialPose = swerveSubsystem.getRobotPosition();
        wrappedDriveCommand = FollowPathCommand.from(
            swerveSubsystem,
            initialPose,
            List.of(),
            alignPose
        );

        wrappedDriveCommand.schedule();
    }

    /**
     * Cancels the auto-align command.
     */
    public void cancel() {
        wrappedDriveCommand.cancel();
    }
}
