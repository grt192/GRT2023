package frc.robot.commands.dropping;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.positions.PlacePosition;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class AutoAlignToClosestCommand extends InstantCommand {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    private final boolean isRed;

    private AutoAlignCommand wrappedCommand;

    /**
     * Creates an auto-align-to-closest command from a given swerve subsystem, elevator subsystem, and
     * whether the robot is currently on the red team.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether the robot is on the red team.
     */
    public AutoAlignToClosestCommand(
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

        // Target the closest place position.
        PlacePosition targetPlacePosition = Collections.min(
            Arrays.asList(PlacePosition.values()),
            Comparator.comparing((pos) -> initialPose.getTranslation().getDistance(pos.alignPosition.getPose(isRed).getTranslation()))
        );

        // Schedule auto-align command to align with targeted node.
        wrappedCommand = new AutoAlignCommand(swerveSubsystem, tiltedElevatorSubsystem, targetPlacePosition, isRed);
        wrappedCommand.schedule();
    }

    @Override
    public void cancel() {
        if (wrappedCommand != null) wrappedCommand.cancel();
        super.cancel();
    }
}
