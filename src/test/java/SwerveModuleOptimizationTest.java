import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.swerve.SwerveModule;

public class SwerveModuleOptimizationTest {
    private static final double ACCEPTABLE_ANGLE_DELTA = 1e-15;

    /**
     * This test ensures that the optimize method correctly handles the case where the
     * target angle and current angle are zero (delta = 0).
     * 
     * Current angle: 0, target angle: 0
     * Expected angle: 0, velocity: 1.0 (not flipped)
     */
    @Test
    public void bothZero() {
        runTest(0, 0, 0, false);
    }

    /**
     * This test ensures that the optimize method correctly handles the case where the
     * target angle and current angle are the same (delta = 0).
     * 
     * Current angle: pi/6, target angle: pi/6
     * Expected angle: pi/6, velocity: 1.0 (not flipped)
     */
    @Test
    public void bothSame() {
        runTest(
            Math.PI / 6.0,
            Math.PI / 6.0, 
            Math.PI / 6.0, 
            false
        );
    }

    /**
     * This test ensures that the optimize method does not optimize the module state
     * when the delta is less than 90 degrees.
     * 
     * Current angle: pi/6, target angle: 2pi/6
     * Expected angle: 2pi/6, velocity: 1.0 (not flipped)
     */
    @Test
    public void unoptimizedNowraparound() {
        runTest(
            Math.PI / 6.0,
            2 * Math.PI / 6.0, 
            2 * Math.PI / 6.0, 
            false
        );
    }

    /**
     * This test ensures that the optimize method correctly optimizes the module state
     * when the delta is greater than 90 degrees.
     * 
     * Current angle: pi/6, target angle: 5pi/6
     * Expected angle: -pi/6, velocity: -1.0 (flipped)
     */
    @Test
    public void optimizedNowraparound() {
        runTest(
            Math.PI / 6.0,
            5 * Math.PI / 6.0, 
            -Math.PI / 6.0, 
            true
        );
    }

    /**
     * Runs a unit test with the given parameters.
     * 
     * @param currentAngleRads The current angle, in radians, between [-pi, pi].
     * @param targetAngleRads The target angle, in radians, between [-pi, pi].
     * @param expectedAngleRads The expected angle, in radians, between [-pi, pi].
     * @param expectedOptimized Whether the state is expected to be optimized (velocity flipped).
     */
    private void runTest(double currentAngleRads, double targetAngleRads, double expectedAngleRads, boolean expectedOptimized) {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(targetAngleRads));
        SwerveModuleState optimized = SwerveModule.optimizeModuleState(target, new Rotation2d(currentAngleRads));

        assertEquals(expectedOptimized ? -1.0 : 1.0, optimized.speedMetersPerSecond, 0);
        assertEquals(expectedAngleRads, optimized.angle.getRadians(), ACCEPTABLE_ANGLE_DELTA);
    }
}
