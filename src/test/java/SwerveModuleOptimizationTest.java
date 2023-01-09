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
     * This test ensures that the optimize method correctly performs angle wraparound 
     * (ie. going from pi to 7pi/6 when the target angle is -5pi/6) without optimization.
     * 
     * Current angle: 5pi/6, target angle: -5pi/6
     * Expected angle: 5pi/6 + pi/3 (delta) = 7pi/6, velocity: 1.0 (not flipped)
     */
    @Test
    public void unoptimizedWraparound() {
        runTest(
            5 * Math.PI / 6.0,
            -5 * Math.PI / 6.0, 
            7 * Math.PI / 6.0, 
            false
        );
    }

    /**
     * This test ensures that the optimize method correctly optimizes the module state
     * with angle wraparound (ie. going from pi to 7pi/6 when the target angle is -5pi/6).
     * 
     * Current angle: pi/3, target angle: -5pi/6
     * Expected angle: pi/3 - pi/6 (delta) = pi/6, velocity: -1.0 (flipped)
     */
    @Test
    public void optimizedWraparound() {
        runTest(
            Math.PI / 3.0, 
            -5 * Math.PI / 6.0, 
            Math.PI / 6.0, 
            true
        );
    }

    /**
     * This test ensures that the optimize method both correctly performs angle wraparound 
     * (ie. going from pi to 7pi/6 when the target angle is -5pi/6) without optimization and 
     * correctly applies the delta to an angle outside of (-pi, pi].
     * 
     * Current angle: 29pi/6, target angle: -5pi/6
     * Expected angle: 29pi/6 + pi/3 (delta) = 31pi/6, velocity: 1.0 (not flipped)
     */
    @Test
    public void unoptimizedUnboundedWraparound() {
        runTest(
            29 * Math.PI / 6.0, 
            -5 * Math.PI / 6.0, 
            31 * Math.PI / 6.0, 
            false
        );
    }

    /**
     * This test ensures that the optimize method both correctly optimizes the module state
     * with angle wraparound (ie. going from pi to 7pi/6 when the target angle is -5pi/6) and 
     * correctly applies the delta to an angle outside of (-pi, pi].
     * 
     * Current angle: 26pi/6, target angle: -5pi/6
     * Expected angle: 26pi/6 - pi/6 (delta) = 25pi/6, velocity: -1.0 (flipped)
     */
    @Test
    public void optimizedUnboundedWraparound() {
        runTest(
            26 * Math.PI / 6.0, 
            -5 * Math.PI / 6.0, 
            25 * Math.PI / 6.0, 
            true
        );
    }

    /**
     * Runs a unit test with the given parameters.
     * 
     * @param currentAngleRads The current angle, in radians.
     * @param targetAngleRads The target angle, in radians.
     * @param expectedAngleRads The expected angle, in radians.
     * @param expectedOptimized Whether the state is expected to be optimized (velocity flipped).
     */
    private void runTest(double currentAngleRads, double targetAngleRads, double expectedAngleRads, boolean expectedOptimized) {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(targetAngleRads));
        var optimized = SwerveModule.optimizeModuleState(target, new Rotation2d(currentAngleRads));

        assertEquals(expectedOptimized ? -1.0 : 1.0, optimized.getFirst(), 0);
        assertEquals(expectedAngleRads, optimized.getSecond(), ACCEPTABLE_ANGLE_DELTA);
    }
}
