package frc.robot.commands.dropping;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        double currentExtensionMeters = tiltedElevatorSubsystem.getExtensionMeters();

        // Get closest place pose to align with
        PlacePosition targetPlacePosition = getClosestPlacePosition(initialPose, currentExtensionMeters, isRed);
        System.out.println("Aligning with " + targetPlacePosition.name());

        // Schedule command to align with targeted node
        wrappedCommand = new AlignToNodeCommand(swerveSubsystem, tiltedElevatorSubsystem, targetPlacePosition, isRed);
        wrappedCommand.schedule();
    }

    /**
     * Gets the closest place position given the current robot position and extension of the elevator. Positions are
     * initially compared with their distance to the robot, and ties (eg. between HIGH and MID positions) are broken
     * using elevator extension.
     * 
     * @param robotPose The current pose of the robot.
     * @param elevatorExtensionMeters The current extension, in meters, of the elevator.
     * @param isRed Whether the robot is on the red team.
     * @return The closest `PlacePosition`.
     */
    public static PlacePosition getClosestPlacePosition(Pose2d robotPose, double elevatorExtensionMeters, boolean isRed) {
        return Collections.min(
            Arrays.asList(PlacePosition.values()),
            Comparator.comparing((PlacePosition pos) -> {
                Translation2d targetTranslation = pos.alignPosition.getPose(isRed).getTranslation();
                return robotPose.getTranslation().getDistance(targetTranslation);
            }).thenComparing((PlacePosition pos) -> {
                double targetExtensionMeters =  pos.elevatorState.getExtension(OffsetState.DROPPING, true);
                return Math.abs(targetExtensionMeters - elevatorExtensionMeters);
            })
        );
    }

    @Override
    public void cancel() {
        if (wrappedCommand != null) wrappedCommand.cancel();
        super.cancel();
    }
}
