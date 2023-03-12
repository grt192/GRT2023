package frc.robot.commands.dropping;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class AutoAlignGrid {
    private final AlignToNodeCommand[] setTargetCommands;

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
        PlacePosition[] positions = PlacePosition.values();
        setTargetCommands = new AlignToNodeCommand[positions.length];

        for (int i = 0; i < positions.length; i++) {
            PlacePosition position = positions[i];

            AlignToNodeCommand command = new AlignToNodeCommand(swerveSubsystem, tiltedElevatorSubsystem, position, isRed);
            setTargetCommands[i] = command;

            // Add command to grid
            shuffleboardTab.add(position.name(), command).withPosition(
                getShuffleboardColumn(position),
                getShuffleboardRow(position)
            );
        }
    }

    /**
     * Cancels all currently scheduled auto-align commands.
     */
    public void cancelAll() {
        for (AlignToNodeCommand command : setTargetCommands) {
            command.cancel();
        }
    }

    private static int getShuffleboardRow(PlacePosition position) {
        return switch (position) {
            case A1_HIGH, A2_HIGH, A3_HIGH, B1_HIGH, B2_HIGH, B3_HIGH, C1_HIGH, C2_HIGH, C3_HIGH -> 0;
            case A1_MID, A2_MID, A3_MID, B1_MID, B2_MID, B3_MID, C1_MID, C2_MID, C3_MID -> 1;
            case A1_HYBRID, A2_HYBRID, A3_HYBRID, B1_HYBRID, B2_HYBRID, B3_HYBRID, C1_HYBRID, C2_HYBRID, C3_HYBRID -> 2;
            default -> throw new RuntimeException("Position not found!");
        };
    }

    private static int getShuffleboardColumn(PlacePosition position) {
        return switch (position) {
            case A1_HIGH, A1_MID, A1_HYBRID -> 0;
            case A2_HIGH, A2_MID, A2_HYBRID -> 1;
            case A3_HIGH, A3_MID, A3_HYBRID -> 2;
            case B1_HIGH, B1_MID, B1_HYBRID -> 3;
            case B2_HIGH, B2_MID, B2_HYBRID -> 4;
            case B3_HIGH, B3_MID, B3_HYBRID -> 5;
            case C1_HIGH, C1_MID, C1_HYBRID -> 6;
            case C2_HIGH, C2_MID, C2_HYBRID -> 7;
            case C3_HIGH, C3_MID, C3_HYBRID -> 8;
            default -> throw new RuntimeException("Position not found!");
        };
    }
}
