package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.positions.FieldPosition;
import frc.robot.positions.PlacePosition;
import frc.robot.positions.PlacePosition.PlaceState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import frc.robot.subsystems.tiltedelevator.TiltedElevatorSubsystem;

public class PreloadedOnlyAutonSequence extends BaseAutonSequence {
    private static final FieldPosition INITIAL_POSE = FieldPosition.A2_INIT;
    private static final PlacePosition PLACE_POSE = PlacePosition.A2HIGH;

    /**
     * Preloaded-only auton sequence. This only places the preloaded game piece and
     * does not taxi.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem The tilted elevator subsystem.
     * @param isRed Whether this is a red auton path.
     */
    public PreloadedOnlyAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        boolean isRed // TODO: better way of passing this
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem, INITIAL_POSE.getPose(isRed)); // TODO: better

        Pose2d initialPose = INITIAL_POSE.getPose(isRed);
        PlaceState placeState1 = PLACE_POSE.getPlaceState(isRed);

        addCommands(
            // Place preloaded game piece
            goAndPlace(initialPose, placeState1)
        );
    }
}
