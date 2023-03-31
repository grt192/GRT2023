package frc.robot.commands.dropping;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.swerve.GoToPointCommand;
import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;
import frc.robot.subsystems.tiltedelevator.ElevatorState.OffsetState;

/**
 * An auto-align command that maintains a target `PlacePosition`. When scheduled, this command sets the target
 * to the closest `PlacePosition` and aligns with it. This command also creates a grid of buttons on Shuffleboard
 * to set the target to any given node. `alignLeft()` and `alignRight()` can be used to shift the target left or
 * right.
 */
public class AutoAlignCommand extends InstantCommand {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;
    private boolean isRed;

    private final InstantCommand[] setTargetCommands;
    private final HashMap<PlacePosition, ComplexWidget> commandEntries;
    private final HashMap<FieldPosition, SimpleWidget> booleanEntries;

    private volatile PlacePosition targetPlacePosition;
    private volatile Command wrappedAlignCommand;
    private volatile GoToPointCommand wrappedGoForwardCommand;

    private final ShuffleboardTab shuffleboardTab;

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

        // Initialize shuffleboard alignment buttons
        shuffleboardTab = Shuffleboard.getTab("Driver");
        commandEntries = new HashMap<>();
        booleanEntries = new HashMap<>();

        PlacePosition[] positions = PlacePosition.values();
        setTargetCommands = new InstantCommand[positions.length];

        for (int i = 0; i < positions.length; i++) {
            PlacePosition position = positions[i];

            // When pressed, set the target place position to the given value and align to the node.
            InstantCommand command = new InstantCommand(() -> scheduleAlignCommandWith(position, true), swerveSubsystem, tiltedElevatorSubsystem);
            setTargetCommands[i] = command;

            // Add command and boolean indicator to grid
            ComplexWidget commandWidget = shuffleboardTab.add(position.name(), command);
            commandEntries.put(position, commandWidget);
            if (!booleanEntries.containsKey(position.placePosition)) booleanEntries.put(
                position.placePosition,
                shuffleboardTab.add(position.placePosition.name(), false)
            );

            updateShuffleboardPositions();
        }
    }

    @Override
    public void initialize() {
        // Align with closest place position
        scheduleAlignCommandWithClosest();
    }

    /**
     * Aligns to the node adjacent to the current target in the direction of C3, or the closest node
     * if no previous target is stored. This is a no-op if the selected node is already C3.
     */
    public void alignTowardsC3() {
        if (targetPlacePosition == null) {
            scheduleAlignCommandWithClosest();
            return;
        };
        if (targetPlacePosition.placePosition == FieldPosition.C3) return;

        // TODO: better way of doing this?
        ElevatorState currentElevatorState = tiltedElevatorSubsystem.getState();
        scheduleAlignCommandWith(switch (targetPlacePosition.placePosition) {
            case A1 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.A2_HYBRID, PlacePosition.A2_MID, PlacePosition.A2_HIGH);
            case A2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.A3_HYBRID, PlacePosition.A3_MID, PlacePosition.A3_HIGH);
            case A3 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B1_HYBRID, PlacePosition.B1_MID, PlacePosition.B1_HIGH);
            case B1 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B2_HYBRID, PlacePosition.B2_MID, PlacePosition.B2_HIGH);
            case B2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B3_HYBRID, PlacePosition.B3_MID, PlacePosition.B3_HIGH);
            case B3 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.C1_HYBRID, PlacePosition.C1_MID, PlacePosition.C1_HIGH);
            case C1 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.C2_HYBRID, PlacePosition.C2_MID, PlacePosition.C2_HIGH);
            case C2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.C3_HYBRID, PlacePosition.C3_MID, PlacePosition.C3_HIGH);
            default -> throw new RuntimeException("Position not found!");
        });
    }

    /**
     * Aligns to the node adjacent to the current target in the direction of A1, or the closest node
     * if no previous target is stored. This is a no-op if the selected node is already A1.
     */
    public void alignTowardsA1() {
        if (targetPlacePosition == null) {
            scheduleAlignCommandWithClosest();
            return;
        };
        if (targetPlacePosition.placePosition == FieldPosition.A1) return;

        // TODO: better way of doing this?
        ElevatorState currentElevatorState = tiltedElevatorSubsystem.getState();
        scheduleAlignCommandWith(switch (targetPlacePosition.placePosition) {
            case A2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.A1_HYBRID, PlacePosition.A1_MID, PlacePosition.A1_HIGH);
            case A3 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.A2_HYBRID, PlacePosition.A2_MID, PlacePosition.A2_HIGH);
            case B1 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.A3_HYBRID, PlacePosition.A3_MID, PlacePosition.A3_HIGH);
            case B2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B1_HYBRID, PlacePosition.B1_MID, PlacePosition.B1_HIGH);
            case B3 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B2_HYBRID, PlacePosition.B2_MID, PlacePosition.B2_HIGH);
            case C1 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.B3_HYBRID, PlacePosition.B3_MID, PlacePosition.B3_HIGH);
            case C2 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.C1_HYBRID, PlacePosition.C1_MID, PlacePosition.C1_HIGH);
            case C3 -> elevatorStateToPlacePosition(currentElevatorState, PlacePosition.C2_HYBRID, PlacePosition.C2_MID, PlacePosition.C2_HIGH);
            default -> throw new RuntimeException("Position not found!");
        });
    }

    /**
     * Aligns to the node to the left of the current target, from the perspective of the drivers.
     */
    public void alignLeft() {
        if (isRed) alignTowardsA1();
        else alignTowardsC3();
    }

    /**
     * Aligns to the node to the right of the current target, from the perspective of the drivers.
     */
    public void alignRight() {
        if (isRed) alignTowardsC3();
        else alignTowardsA1();
    }

    /**
     * Drives forward against the grid to place. Does nothing if there isn't a target place position.
     */
    public void driveForwardToPlace() {
        if (wrappedGoForwardCommand == null) return;
        if (wrappedAlignCommand != null) wrappedAlignCommand.cancel();
        wrappedGoForwardCommand.schedule();
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
     * @param elevatorState The current state of the elevator.
     * @param isRed Whether the robot is on the red team.
     * @return The closest `PlacePosition`.
     */
    public static PlacePosition getClosestPlacePosition(Pose2d robotPose, ElevatorState elevatorState, boolean isRed) {
        return getClosestPlacePositionOf(
            robotPose, elevatorState, isRed,
            Arrays.asList(PlacePosition.values())
        );
    }

    /**
     * Gets the closest cube place position given the current robot position and extension of the elevator. Positions are
     * initially compared with their distance to the robot, and ties (eg. between HIGH and MID positions) are broken
     * using elevator extension.
     * 
     * @param robotPose The current pose of the robot.
     * @param elevatorState The current state of the elevator.
     * @param isRed Whether the robot is on the red team.
     * @return The closest cube `PlacePosition`.
     */
    public static PlacePosition getClosestCubePlacePosition(Pose2d robotPose, ElevatorState elevatorState, boolean isRed) {
        return getClosestPlacePositionOf(
            robotPose, elevatorState, isRed,
            PlacePosition.getCubePositions()
        );
    }

    /**
     * Gets the closest place position of the given collection of `PlacePosition`s given the current robot position
     * and extension of the elevator. Positions are initially compared with their distance to the robot, and ties
     * (eg. between HIGH and MID positions) are broken using elevator extension.
     * 
     * @param robotPose The current pose of the robot.
     * @param elevatorState The current state of the elevator.
     * @param isRed Whether the robot is on the red team.
     * @param positions The `PlacePosition`s to get the closest of.
     * @return The closest `PlacePosition` of the given collection.
     */
    private static PlacePosition getClosestPlacePositionOf(
        Pose2d robotPose, ElevatorState elevatorState, boolean isRed,
        Collection<PlacePosition> positions
    ) {
        double elevatorExtensionMeters = elevatorState.getExtension(OffsetState.DEFAULT, true);

        return Collections.min(
            positions,
            Comparator.comparing((PlacePosition pos) -> {
                Translation2d targetTranslation = pos.alignPosition.getPose(isRed).getTranslation();
                return robotPose.getTranslation().getDistance(targetTranslation);
            }).thenComparing((PlacePosition pos) -> {
                double targetExtensionMeters =  pos.elevatorState.getExtension(OffsetState.DEFAULT, true);
                return Math.abs(targetExtensionMeters - elevatorExtensionMeters);
            })
        );
    }

    /**
     * Schedules a wrapped `AlignToNodeCommand` to align the robot with the selected node.
     * If an align command was previously scheduled, cancel it before scheduling another.
     * This method also toggles the given boolean entries on shuffleboard to display which
     * node it is aligning with.
     * 
     * @param newPosition The new `PlacePosition` to align with.
     * @param driveForwardAfterwards TODO
     */
    private void scheduleAlignCommandWith(PlacePosition newPosition, boolean driveForwardAfterwards) {
        if (targetPlacePosition != null) booleanEntries.get(targetPlacePosition.placePosition).getEntry().setBoolean(false);
        booleanEntries.get(newPosition.placePosition).getEntry().setBoolean(true);
        targetPlacePosition = newPosition;

        if (wrappedGoForwardCommand != null) wrappedGoForwardCommand.cancel();
        if (wrappedAlignCommand != null) wrappedAlignCommand.cancel();

        wrappedGoForwardCommand = new GoToPointCommand(swerveSubsystem, targetPlacePosition.placePosition.getPose(isRed));

        // If we automatically drive forward afterwards, the align command is a composition that automatically schedules the
        // drive forward command. Otherwise, just align.
        if (driveForwardAfterwards) {
            wrappedAlignCommand = new AlignToNodeCommand(swerveSubsystem, tiltedElevatorSubsystem, targetPlacePosition, isRed).andThen(
                new GoToPointCommand(swerveSubsystem, targetPlacePosition.placePosition.getPose(isRed))
            );
        } else {
            wrappedAlignCommand = new AlignToNodeCommand(swerveSubsystem, tiltedElevatorSubsystem, targetPlacePosition, isRed);
        }

        // System.out.println("Aligning with " + targetPlacePosition.name());
        wrappedAlignCommand.schedule();

        // Lock parallel to the grid for better strafing
        swerveSubsystem.setChargingStationLocked(true);
    }

    private void scheduleAlignCommandWith(PlacePosition newPosition) {
        scheduleAlignCommandWith(newPosition, false);
    }

    /**
     * Schedules a wrapped `AlignToNodeCommand` to align the robot with the closest cube node.
     * See {@link #scheduleAlignCommandWith(PlacePosition newPosition) scheduleAlignCommandWith}.
     */
    private void scheduleAlignCommandWithClosest() {
        Pose2d initialPose = swerveSubsystem.getRobotPosition();
        ElevatorState currentElevatorState = tiltedElevatorSubsystem.getState();
        scheduleAlignCommandWith(getClosestCubePlacePosition(initialPose, currentElevatorState, isRed));
    }

    /**
     * Cancels this command, the wrapped align command if it is scheduled, and
     * any scheduled Shuffleboard button command.
     */
    @Override
    public void cancel() {
        if (targetPlacePosition != null) booleanEntries.get(targetPlacePosition.placePosition).getEntry().setBoolean(false);

        if (wrappedAlignCommand != null) wrappedAlignCommand.cancel();
        if (wrappedGoForwardCommand != null) wrappedGoForwardCommand.cancel();
        for (InstantCommand command : setTargetCommands) {
            command.cancel();
        }
        super.cancel();

        // Revert strafing lock and target place position
        swerveSubsystem.setChargingStationLocked(false);
        targetPlacePosition = null;
    }

    /**
     * Sets the alliance of this auto align command.
     * @param isRed Whether the robot is on the red team. If false, aligns to blue nodes.
     */
    public void setIsRed(boolean isRed) {
        this.isRed = isRed;
        updateShuffleboardPositions();
    }

    /**
     * Updates the positions of the shuffleboard entries, depending on
     * whether we are on the red team.
     */
    private void updateShuffleboardPositions() {
        commandEntries.forEach((position, widget) -> {
            widget.withPosition(
                getShuffleboardColumn(position),
                getShuffleboardRow(position)
            );
        });

        booleanEntries.forEach((position, widget) -> {
            widget.withPosition(getShuffleboardColumn(position), 3);
        });
    }

    private int getShuffleboardRow(PlacePosition position) {
        return switch (position) {
            case A1_HIGH, A2_HIGH, A3_HIGH, B1_HIGH, B2_HIGH, B3_HIGH, C1_HIGH, C2_HIGH, C3_HIGH -> 0;
            case A1_MID, A2_MID, A3_MID, B1_MID, B2_MID, B3_MID, C1_MID, C2_MID, C3_MID -> 1;
            case A1_HYBRID, A2_HYBRID, A3_HYBRID, B1_HYBRID, B2_HYBRID, B3_HYBRID, C1_HYBRID, C2_HYBRID, C3_HYBRID -> 2;
            default -> throw new RuntimeException("Position not found!");
        };
    }

    private int getShuffleboardColumn(PlacePosition position) {
        return getShuffleboardColumn(position.placePosition);
    }

    private int getShuffleboardColumn(FieldPosition position) {
        int bluePosition = switch (position) {
            case A1 -> 8;
            case A2 -> 7;
            case A3 -> 6;
            case B1 -> 5;
            case B2 -> 4;
            case B3 -> 3;
            case C1 -> 2;
            case C2 -> 1;
            case C3 -> 0;
            default -> throw new RuntimeException("Position not found!");
        };

        return isRed ? (8 - bluePosition) : bluePosition;
    }
}
