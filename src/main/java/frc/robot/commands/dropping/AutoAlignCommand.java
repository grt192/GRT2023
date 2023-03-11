package frc.robot.commands.dropping;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class AutoAlignCommand extends InstantCommand {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    private final PlacePosition targetPlacePosition;
    private final boolean isRed;

    private FollowPathCommand wrappedDriveCommand;

    /**
     * Creates an auto-align command from a given swerve subsystem, elevator subsystem,  and
     * whether the robot is currently on the red team. This command will align to the closest node.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether the robot is on the red team.
     */
    public AutoAlignCommand(
        BaseSwerveSubsystem swerveSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed
    ) {
        this(swerveSubsystem, tiltedElevatorSubsystem, null, isRed);
    }

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
        PlacePosition targetPlacePosition, boolean isRed
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        this.targetPlacePosition = targetPlacePosition;
        this.isRed = isRed;

        addRequirements(swerveSubsystem, tiltedElevatorSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d initialPose = swerveSubsystem.getRobotPosition();

        // Align to the target place position if it was provided, or the closest position otherwise.
        PlacePosition position = targetPlacePosition != null ? targetPlacePosition : Collections.min(
            Arrays.asList(PlacePosition.values()),
            Comparator.comparing((pos) -> initialPose.getTranslation().getDistance(pos.alignPosition.getPose(isRed).getTranslation()))
        );

        tiltedElevatorSubsystem.setState(position.elevatorState);

        // Schedule follow path command to align with node
        wrappedDriveCommand = FollowPathCommand.from(
            swerveSubsystem,
            initialPose,
            List.of(),
            position.alignPosition.getPose(isRed)
        );

        wrappedDriveCommand.schedule();
    }

    @Override
    public void cancel() {
        if (wrappedDriveCommand != null) wrappedDriveCommand.cancel();
        super.cancel();
    }
}
