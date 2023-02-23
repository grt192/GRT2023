import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;

import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.swerve.FollowPathCommand;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class WheelHeadingAngleTest {
    private static final double ACCEPTABLE_ANGLE_DELTA = 1e-15;

    /**
     * Ensures that wheel headings are correctly averaged with two positive angles.
     * A: 30 degrees, B: 60 degrees, average: 45 degrees
     */
    @Test
    public void positiveNoWraparound() {
        Rotation2d average = FollowPathCommand.averageWheelHeadings(
            Rotation2d.fromDegrees(30),
            Rotation2d.fromDegrees(60)
        );
        assertEquals(average.getRadians(), Math.PI / 4, ACCEPTABLE_ANGLE_DELTA);
    }

    /**
     * Ensures that wheel headings are correctly averaged with two negative angles.
     * A: -30 degrees, B: -60 degrees, average: -45 degrees
     */
    @Test
    public void negativeNoWraparound() {
        Rotation2d average = FollowPathCommand.averageWheelHeadings(
            Rotation2d.fromDegrees(-30),
            Rotation2d.fromDegrees(-60)
        );
        assertEquals(average.getRadians(), -Math.PI / 4, ACCEPTABLE_ANGLE_DELTA);
    }

    /**
     * Ensures that wheel headings are correctly averaged with a positive and negative
     * (but not wrapped) angle.
     * 
     * A: -45 degrees, B: 45 degrees, average: 0 degrees
     */
    @Test
    public void changeSignsNoWraparound() {
        Rotation2d average = FollowPathCommand.averageWheelHeadings(
            Rotation2d.fromDegrees(-45),
            Rotation2d.fromDegrees(45)
        );
        assertEquals(average.getRadians(), 0.0, ACCEPTABLE_ANGLE_DELTA);
    }

    /**
     * Ensures that wheel headings are correctly wrapped when delta > 180.
     * A: -120 degrees, B: 180 degrees
     * no wraparound average (incorrect): 30 degrees, actual average: 330 degrees (-60 degrees)
     */
    @Test
    public void changeSignsWraparound() {
        Rotation2d average = FollowPathCommand.averageWheelHeadings(
            Rotation2d.fromDegrees(-120),
            Rotation2d.fromDegrees(180)
        );
        assertEquals(average.getRadians(), -Math.PI / 3, ACCEPTABLE_ANGLE_DELTA);
    }
}
