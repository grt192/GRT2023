package frc.robot.commands.sequences;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.BalancerCommand;
import frc.robot.commands.grabber.AidenIntakeCommand;
import frc.robot.commands.grabber.AidenPlaceCommand;
import frc.robot.commands.mover.JulianLevelCommand;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem;
import frc.robot.subsystems.TiltedElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * Auton Command Non-Balancing Sequence. Robot places pre-loaded piece, exits community to grab another, then places that peice
 */
public abstract class BaseAutonSequence extends SequentialCommandGroup {
    private final BaseSwerveSubsystem swerveSubsystem;
    private final RollerSubsystem rollerSubsystem;
    private final TiltedElevatorSubsystem tiltedElevatorSubsystem;

    public static double BLUE_INITX = 70.007;
    public static double RED_INITX = 581.072;

    private static double RED_PLACEX = 578.737;
    private static double BLUE_PLACEX = 72.013;

    //positions for placing gamepieces
    //all positions from center of robot
    // ^^^ from center of robot? aren't these field coordinates?
    public enum RedPositions {
        X1(new Pose2d(
            Units.inchesToMeters(RED_PLACEX),
            Units.inchesToMeters(195.55),
            Rotation2d.fromDegrees(0)
        )),
        X2(new Pose2d(
            Units.inchesToMeters(RED_PLACEX),
            Units.inchesToMeters(174.123),
            Rotation2d.fromDegrees(0)
        )),
        X3(new Pose2d(
            Units.inchesToMeters(RED_PLACEX),
            Units.inchesToMeters(152.123),
            Rotation2d.fromDegrees(0)
        )),
        X4(new Pose2d(
            Units.inchesToMeters(RED_PLACEX),
            Units.inchesToMeters(129.75),
            Rotation2d.fromDegrees(0)
        )),
        X5(new Pose2d(
            Units.inchesToMeters(RED_PLACEX),
            Units.inchesToMeters(107.801),
            Rotation2d.fromDegrees(0)
        )),
        X6(new Pose2d(
            Units.inchesToMeters(RED_PLACEX),
            Units.inchesToMeters(86.149),
            Rotation2d.fromDegrees(0)
        )),
        X7(new Pose2d(
            Units.inchesToMeters(RED_PLACEX),
            Units.inchesToMeters(64.818),
            Rotation2d.fromDegrees(0)
        )),
        X8(new Pose2d(
            Units.inchesToMeters(RED_PLACEX),
            Units.inchesToMeters(41.761),
            Rotation2d.fromDegrees(0)
        )),
        X9(new Pose2d(
            Units.inchesToMeters(RED_PLACEX),
            Units.inchesToMeters(20.016),
            Rotation2d.fromDegrees(0)
        )),

        PIECE1(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE2(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE3(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE4(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        BALANCE_INIT(new Pose2d(
            Units.inchesToMeters(RED_INITX),
            Units.inchesToMeters(107.638),
            Rotation2d.fromDegrees(0)
        ));

        public Pose2d position;

        private RedPositions(Pose2d position){
            this.position = position;
        }
    } 

    //positions for placing gamepieces
    public enum BluePositions {
        X1(new Pose2d(
            Units.inchesToMeters(BLUE_PLACEX),
            Units.inchesToMeters(195.55),
            Rotation2d.fromDegrees(180)
        )),
        X2(new Pose2d(
            Units.inchesToMeters(BLUE_PLACEX),
            Units.inchesToMeters(174.123),
            Rotation2d.fromDegrees(180)
        )),
        X3(new Pose2d(
            Units.inchesToMeters(BLUE_PLACEX),
            Units.inchesToMeters(152.123),
            Rotation2d.fromDegrees(180)
        )),
        X4(new Pose2d(
            Units.inchesToMeters(BLUE_PLACEX),
            Units.inchesToMeters(129.75),
            Rotation2d.fromDegrees(180)
        )),
        X5(new Pose2d(
            Units.inchesToMeters(BLUE_PLACEX),
            Units.inchesToMeters(107.801),
            Rotation2d.fromDegrees(180)
        )),
        X6(new Pose2d(
            Units.inchesToMeters(BLUE_PLACEX),
            Units.inchesToMeters(86.149),
            Rotation2d.fromDegrees(180)
        )),
        X7(new Pose2d(
            Units.inchesToMeters(BLUE_PLACEX),
            Units.inchesToMeters(64.818),
            Rotation2d.fromDegrees(180)
        )),
        X8(new Pose2d(
            Units.inchesToMeters(BLUE_PLACEX),
            Units.inchesToMeters(41.761),
            Rotation2d.fromDegrees(180)
        )),
        X9(new Pose2d(
            Units.inchesToMeters(BLUE_PLACEX),
            Units.inchesToMeters(20.016),
            Rotation2d.fromDegrees(180)
        )),

        PIECE1(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE2(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE3(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        PIECE4(new Pose2d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Rotation2d.fromDegrees(0)
        )),
        BALANCE_INIT(new Pose2d(
            Units.inchesToMeters(BLUE_INITX),
            Units.inchesToMeters(107.638),
            Rotation2d.fromDegrees(180)
        ));

        public Pose2d position;

        private BluePositions(Pose2d position){
            this.position = position;
        }
    }

    /**
     * non balancing TOP auton sequence
     * @param robotContainer
     * @param initialPose initPose
     * @param midPose1 midPose1
     * @param midPose2 midPose2
     * @param midpose3 misPose3
     * @param placePose placepose of first gamepeice
     * @param grabPose grabpose of 2nd gamepiece
     * @param placePose2 placepose of 2nd gamepiece
     * @param elevatorPosition The height to place the 1st game piece.
     * @param elevatorPosition2 The height to place the 2nd game piece.
     */
    public BaseAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midpose3, 
        Pose2d placePose, Pose2d grabPose, Pose2d placePose2, 
        ElevatorState elevatorPosition, ElevatorState elevatorPosition2
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            // Place preloaded game piece
            goAndPlaceTop(initialPose, midPose1, midPose2, midpose3, placePose, elevatorPosition),
            // Go and grab 2nd piece
            goAndGrabTop(placePose, midPose1, midPose2, midpose3, grabPose), 
            // Go and place grabbed piece
            goAndPlaceTop(grabPose, midPose1, midPose2, midpose3, placePose2, elevatorPosition2)
        );
    }

    /**
     * non balancing BOTTOM auton sequence
     * @param robotContainer
     * @param initialPose initPose
     * @param midPose1 midPose1
     * @param midPose2 midPose2
     * @param placePose placepose of first gamepeice
     * @param grabPose grabpose of 2nd gamepiece
     * @param placePose2 placepose of 2nd gamepiece
     * @param elevatorPosition The height to place the 1st game piece.
     * @param elevatorPosition2 The height to place the 2nd game piece.
     */
    public BaseAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, 
        Pose2d placePose, Pose2d grabPose, Pose2d placePose2, 
        ElevatorState elevatorPosition, ElevatorState elevatorPosition2
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            // Place preloaded game piece
            goAndPlaceBottom(initialPose, midPose1, midPose2, placePose, elevatorPosition),
            // Go and grab 2nd piece
            goAndGrabBottom(placePose, midPose1, midPose2, grabPose),
            // Go and place grabbed piece
            goAndPlaceBottom(grabPose, midPose1, midPose2, placePose2, elevatorPosition2)
        );
    }

    /**
     * BALANCING auton sequence
     * @param robotContainer
     * @param initialPose initPose
     * @param placePose placepose of first gamepeice
     * @param outsidePose outside pose to gt OUTSIDE of the community (for extra points)
     * @param elevatorPosition The height to place the 1st game piece.
     */
    public BaseAutonSequence(
        BaseSwerveSubsystem swerveSubsystem, RollerSubsystem rollerSubsystem, TiltedElevatorSubsystem tiltedElevatorSubsystem,
        Pose2d initialPose, Pose2d placePose, Pose2d outsidePose, 
        ElevatorState elevatorPosition
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSubsystem = rollerSubsystem;
        this.tiltedElevatorSubsystem = tiltedElevatorSubsystem;

        addRequirements(swerveSubsystem, rollerSubsystem, tiltedElevatorSubsystem);

        addCommands(
            // Place preloaded game piece
            goAndPlace(initialPose, placePose, elevatorPosition),
            // Go out of community
            new FollowPathCommand(swerveSubsystem, placePose, List.of(), outsidePose),
            // Go and balance on charging station
            new BalancerCommand(swerveSubsystem)
        );
    }

    /**
     * Goes to a position and intakes a game piece. Not used rn tho lol
     * @param intialPose The initial pose of the robot.
     * @param finalPose The destination pose of the robot.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    private Command goAndGrab(Pose2d initialPose, Pose2d finalPose) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), finalPose)
            .alongWith(new JulianLevelCommand(tiltedElevatorSubsystem, ElevatorState.GROUND))
            .alongWith(new AidenIntakeCommand(rollerSubsystem));
    }

    /**
     * Goes to a position and places the currently held game piece.
     * @param intialPose The initial pose of the robot.
     * @param finalPose The destination pose of the robot.
     * @param elevatorPosition The target position of the mover.
     * @return The `SequentialCommandGroup` representing running the commands in order.
     */
    private Command goAndPlace(Pose2d initialPose, Pose2d finalPose, ElevatorState elevatorPosition) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), finalPose)
            .alongWith(new JulianLevelCommand(tiltedElevatorSubsystem, elevatorPosition)) // or .alongWith()?
            .andThen(new AidenPlaceCommand(rollerSubsystem));
    }

    /**
     * Goes to a positon to grab gamepiece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 (usually the pose that turns the robot 90)
     * @param midPose3 (usually the pose that turns the robot 90)
     * @param finalPose Desitnation position of robot
     * @return
     */
    private Command goAndGrabTop(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midPose3, Pose2d finalPose) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(new FollowPathCommand(swerveSubsystem, midPose2, List.of(), midPose3))
            .andThen(goAndGrab(midPose3, finalPose));
    }

    /**
     * Goes to a positon to place gamepiece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param midPose3 (usually the pose that turns the robot 90)
     * @param finalPose Desitnation position of robot
     * @return
     */
    private Command goAndPlaceTop(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d midPose3, Pose2d finalPose, ElevatorState elevatorPosition) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(new FollowPathCommand(swerveSubsystem, midPose2, List.of(), midPose3))
            .andThen(goAndPlace(midPose3, finalPose, elevatorPosition));
    }

    /**
     * Goes to a positon to place a game piece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param finalPose Desitnation position of robot
     * @return
     */
    private Command goAndGrabBottom(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d finalPose) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(goAndGrab(midPose2, finalPose));
    }

    /**
     * Goes to a positon to place a game piece, avoiding the charging station and only turning 90 degrees.
     * @param initialPose The initial pose of the robot
     * @param midPose1 Middle pose 1
     * @param midPose2 Middle pose 2 
     * @param finalPose Desitnation position of robot
     * @return
     */
    private Command goAndPlaceBottom(Pose2d initialPose, Pose2d midPose1, Pose2d midPose2, Pose2d finalPose, ElevatorState elevatorPosition) {
        return new FollowPathCommand(swerveSubsystem, initialPose, List.of(), midPose1)
            .andThen(new FollowPathCommand(swerveSubsystem, midPose1, List.of(), midPose2))
            .andThen(goAndPlace(midPose2, finalPose, elevatorPosition));
    }
}
