package frc.robot.commands.dropping;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState.OffsetState;

public class AutoAlignCommand extends InstantCommand {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    private final boolean isRed;

    private AlignToNodeCommand wrappedCommand;

    /**
     * Creates an auto-align command from a given swerve subsystem, elevator subsystem, and
     * whether the robot is currently on the red team. This command is an `InstantCommand` that
     * schedules an `AlignToNodeCommand` corresponding to the closest node to the current robot
     * position.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether the robot is on the red team.
     */
    public AutoAlignCommand(
        BaseSwerveSubsystem swerveSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        this.isRed = isRed;

        addRequirements(swerveSubsystem, tiltedElevatorSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d initialPose = swerveSubsystem.getRobotPosition();

        // Target the closest place position. If two place positions are equally close (eg. HIGH and MID), align with
        // the one that requires less elevator movement.
        PlacePosition targetPlacePosition = Collections.min(
            Arrays.asList(PlacePosition.values()),
            Comparator.comparing((PlacePosition pos) -> {
                return initialPose.getTranslation().getDistance(pos.alignPosition.getPose(isRed).getTranslation());
            }).thenComparing((PlacePosition pos) -> {
                double currentExtensionMeters = tiltedElevatorSubsystem.getExtensionMeters();
                double targetExtensionMeters =  pos.elevatorState.getExtension(OffsetState.DROPPING, true);
                return Math.abs(targetExtensionMeters - currentExtensionMeters);
            })
        );

        System.out.println("Aligning with " + targetPlacePosition.name());

        // Schedule command to align with targeted node.
        wrappedCommand = new AlignToNodeCommand(swerveSubsystem, tiltedElevatorSubsystem, targetPlacePosition, isRed);
        wrappedCommand.schedule();
    }

    @Override
    public void cancel() {
        if (wrappedCommand != null) wrappedCommand.cancel();
        super.cancel();
    }
}
