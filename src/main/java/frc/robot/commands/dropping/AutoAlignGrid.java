package frc.robot.commands.dropping;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class AutoAlignGrid {
    private final AutoAlignToClosestCommand alignToClosestCommand;
    private final AutoAlignCommand[] setTargetCommands;

    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Grid");

    /**
     * Creates a grid of `AutoAlignCommand` buttons on Shuffleboard from a given swerve subsystem,
     * tilted elevator subsystem, and boolean indicating whether the robot is on the red team.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether the robot is on the red team.
     */
    public AutoAlignGrid(
        BaseSwerveSubsystem swerveSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        alignToClosestCommand = new AutoAlignToClosestCommand(swerveSubsystem, tiltedElevatorSubsystem, isRed);

        PlacePosition[] positions = PlacePosition.values();
        setTargetCommands = new AutoAlignCommand[positions.length];

        for (int i = 0; i < positions.length; i++) {
            PlacePosition position = positions[i];

            AutoAlignCommand command = new AutoAlignCommand(swerveSubsystem, tiltedElevatorSubsystem, position, isRed);
            setTargetCommands[i] = command;

            // Add command to grid
            shuffleboardTab.add(position.name(), command).withPosition(
                getShuffleboardColumn(position),
                getShuffleboardRow(position)
            );
        }
    }

    /**
     * Gets the command to align to the closest node.
     * @return The command to align to the closest node.
     */
    public AutoAlignToClosestCommand getAlignToClosestCommand() {
        return alignToClosestCommand;
    }

    /**
     * Cancels all currently scheduled auto-align commands.
     */
    public void cancelAll() {
        alignToClosestCommand.cancel();
        for (AutoAlignCommand command : setTargetCommands) {
            command.cancel();
        }
    }

    private static int getShuffleboardRow(PlacePosition position) {
        return switch (position) {
            case A1HIGH, A2HIGH, A3HIGH, B1HIGH, B2HIGH, B3HIGH, C1HIGH, C2HIGH, C3HIGH -> 0;
            case A1MID, A2MID, A3MID, B1MID, B2MID, B3MID, C1MID, C2MID, C3MID -> 1;
            default -> throw new RuntimeException("Position not found!");
        };
    }

    private static int getShuffleboardColumn(PlacePosition position) {
        return switch (position) {
            case A1HIGH, A1MID -> 0;
            case A2HIGH, A2MID -> 1;
            case A3HIGH, A3MID -> 2;
            case B1HIGH, B1MID -> 3;
            case B2HIGH, B2MID -> 4;
            case B3HIGH, B3MID -> 5;
            case C1HIGH, C1MID -> 6;
            case C2HIGH, C2MID -> 7;
            case C3HIGH, C3MID -> 8;
            default -> throw new RuntimeException("Position not found!");
        };
    }
}
