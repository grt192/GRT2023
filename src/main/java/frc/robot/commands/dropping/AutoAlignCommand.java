package frc.robot.commands.dropping;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState.OffsetState;

public class AutoAlignCommand extends InstantCommand {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    private final boolean isRed;

    private PlacePosition targetPlacePosition;
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

        // Initialize `targetPlacePosition` to closest place pose
        targetPlacePosition = getClosestPlacePosition(initialPose, currentExtensionMeters, isRed);
        System.out.println("Aligning with " + targetPlacePosition.name());

        scheduleAlignCommand();
    }

    /**
     * Aligns to the node immediately left of the currently selected node.
     * This is a no-op if there isn't yet a node selected, or if the selected node is the left-most node already (A1).
     */
    public void alignLeft() {
        if (targetPlacePosition == null) return;
        if (targetPlacePosition.placePosition == FieldPosition.A1) return;

        // TODO: better way of doing this?
        ElevatorState currentElevatorState = tiltedElevatorSubsystem.getState();
        targetPlacePosition = switch (targetPlacePosition.placePosition) {
            case A2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.A1_HYBRID, PlacePosition.A1_MID, PlacePosition.A1_HIGH);
            case A3 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.A2_HYBRID, PlacePosition.A2_MID, PlacePosition.A2_HIGH);
            case B1 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.A3_HYBRID, PlacePosition.A3_MID, PlacePosition.A3_HIGH);
            case B2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B1_HYBRID, PlacePosition.B1_MID, PlacePosition.B1_HIGH);
            case B3 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B2_HYBRID, PlacePosition.B2_MID, PlacePosition.B2_HIGH);
            case C1 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B3_HYBRID, PlacePosition.B3_MID, PlacePosition.B3_HIGH);
            case C2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.C1_HYBRID, PlacePosition.C1_MID, PlacePosition.C1_HIGH);
            case C3 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.C2_HYBRID, PlacePosition.C2_MID, PlacePosition.C2_HIGH);
            default -> throw new RuntimeException("Position not found!");
        };

        scheduleAlignCommand();
    }

    /**
     * Aligns to the node immediately right of the currently selected node.
     * This is a no-op if there isn't yet a node selected, or if the selected node is the right-most node already (C3).
     */
    public void alignRight() {
        if (targetPlacePosition == null) return;
        if (targetPlacePosition.placePosition == FieldPosition.C3) return;

        // TODO: better way of doing this?
        ElevatorState currentElevatorState = tiltedElevatorSubsystem.getState();
        targetPlacePosition = switch (targetPlacePosition.placePosition) {
            case A1 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.A2_HYBRID, PlacePosition.A2_MID, PlacePosition.A2_HIGH);
            case A2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.A3_HYBRID, PlacePosition.A3_MID, PlacePosition.A3_HIGH);
            case A3 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B1_HYBRID, PlacePosition.B1_MID, PlacePosition.B1_HIGH);
            case B1 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B2_HYBRID, PlacePosition.B2_MID, PlacePosition.B2_HIGH);
            case B2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B3_HYBRID, PlacePosition.B3_MID, PlacePosition.B3_HIGH);
            case B3 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.C1_HYBRID, PlacePosition.C1_MID, PlacePosition.C1_HIGH);
            case C1 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.C2_HYBRID, PlacePosition.C2_MID, PlacePosition.C2_HIGH);
            case C2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.C3_HYBRID, PlacePosition.C3_MID, PlacePosition.C3_HIGH);
            default -> throw new RuntimeException("Position not found!");
        };

        scheduleAlignCommand();
    }

    /**
     * Returns a provided HYBRID, MID, or HIGH place position, depending on the given elevator state.
     * @param elevatorState The current elevator state of the robot.
     * @param hybridPosition The HYBRID position to return.
     * @param midPosition The MID position to return.
     * @param highPosition The HIGH position to return.
     * @return The provided HYBRID, MID, or HIGH place position, depending on the given elevator state.
     */
    private static PlacePosition elevatorStateToPlacePosition(
        ElevatorState elevatorState, PlacePosition hybridPosition, PlacePosition midPosition, PlacePosition highPosition
    ) {
        return switch (elevatorState) {
            case CONE_MID, CUBE_MID -> midPosition;
            case CONE_HIGH, CUBE_HIGH -> highPosition;
            default -> hybridPosition;
        };
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

    /**
     * Schedules a wrapped `AlignToNodeCommand` to align the robot with the selected node.
     * If an align command was previously scheduled, cancel it before scheduling another.
     */
    private void scheduleAlignCommand() {
        if (wrappedCommand != null) wrappedCommand.cancel();
        wrappedCommand = new AlignToNodeCommand(swerveSubsystem, tiltedElevatorSubsystem, targetPlacePosition, isRed);
        wrappedCommand.schedule();
    }
}
