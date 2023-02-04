package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.RobotContainer;
import frc.robot.subsystems.MoverSubsystem.MoverPosition;

/**
 * The top and bottom auton sequences, where the robot deposits its preloaded game piece and picks up
 * and places another.
 */
public class BlueBottomNonBalanceAuton extends BaseAutonSequence {
    
    private static Pose2d MIDPOS1 = (new Pose2d(
        Units.inchesToMeters(94.373),
        Units.inchesToMeters(28.007),
        Rotation2d.fromDegrees(180) ));
    private static Pose2d MIDPOS2 = (new Pose2d(
        Units.inchesToMeters(179.577),
        Units.inchesToMeters(28.007), 
        Rotation2d.fromDegrees(180)));
    private static Pose2d INITPOS = (new Pose2d(
        Units.inchesToMeters(BLUE_INITX),
        Units.inchesToMeters(12.873),
        Rotation2d.fromDegrees(180)
    ));

    /**
     * Constructs an auton sequence from the given parameters.
     * @param robotContainer The robot container.
     * @param placePose1 The robot's pose when placing the first game piece.
     * @param pickPose The robot's pose when intaking the second game piece.
     * @param placePose2 The robot's pose when placing the second game piece.
     * @param height The height of the mover subsystem when placing the first game piece.
     * @param height2 The height of the mover subsystem when placing the second game piece.
     */
    public BlueBottomNonBalanceAuton(
        RobotContainer robotContainer,  
        Pose2d placePose1, 
        Pose2d pickPose, 
        Pose2d placePose2, 
        MoverPosition height, 
        MoverPosition height2
    ) {
        super(robotContainer, INITPOS, MIDPOS1, MIDPOS2, placePose1, pickPose, placePose2, height, height2);
    }
}