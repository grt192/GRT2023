package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

public abstract class BottomAutonSequence extends BaseAutonSequence {
    /**
     * Non-balancing bottom auton sequence.
     * @param swerveSubsystem The swerve subsystem.
     * @param rollerSubsystem The roller subsystem.
     * @param tiltedElevatorSubsystem Thetilted elevator subsystem.
     * @param initialPose The initial pose of the robot.
     * @param midPose1 The first midpose of the sequence. Avoids the charging station.
     * @param midPose2 The second midpose of the sequence. Keeps robot in same orientation to not hit stuff.
     * @param placeState The state of the robot when placing the first game piece (pose and elevator state).
     * @param grabPose The pose to grab the second game piece at.
     * @param placeState2 The state of the robot when placing the second game piece (pose and elevator state).
     */
    public BottomAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, 
        PlacePosition placeState, PiecePosition grabPose, PlacePosition placeState2
    ) {
        super(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            // Place preloaded game piece
            goAndPlaceBottom(initialPose, midPose1, midPose2, placeState),
            // Go and grab 2nd piece
            goAndGrabBottom(placeState.getPose(), midPose1, midPose2, grabPose),
            // Go and place grabbed piece
            goAndPlaceBottom(grabPose.getPose(), midPose1, midPose2, placeState2)
        );
    }
}
