import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.drivetrain.SwerveModule;

public class SwerveModuleOptimizationTest {
    private static final double ACCEPTABLE_ANGLE_DELTA = 1e-15;

    /**
     * Ensures that the relative encoder optimize method correctly handles the case where the target
     * and current angle are both zero (delta = 0).
     * 
     * Current angle: 0, target angle: 0
     * Expected angle: 0. velocity: 1.0 (not flipped)
     */
    @Test
    public void bothZero() {
        runTest(0, 0, 0, false);
    }

    /**
     * Ensures that the relative encoder optimize method correctly handles the case where the target
     * and current angle are both the same (delta = 0).
     * 
     * Current angle: pi/6, target angle: pi/6
     * Expected angle: pi/6. velocity: 1.0 (not flipped)
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
     * Ensures that the relative encoder optimize method correctly optimizes the module state when
     * the delta is greater than 90 degrees.
     * 
     * Current angle: pi/6, target angle: 5pi/6
     * Expected angle: -pi/6. velocity: -1.0 (flipped)
     */
    @Test
    public void optimizedNoWraparound() {
        runTest(
            Math.PI / 6.0,
            5 * Math.PI / 6.0,
            -Math.PI / 6.0,
            true
        );
    }

    /**
     * Ensures that the relative encoder optimize method correctly performs angle wraparound
     * (ie. going from pi to 7pi/6 when the target angle is -5pi/6) without optimization.
     * 
     * Current angle: 5pi/6, target angle: -5pi/6
     * Expected angle: 5pi/6 + pi/3 (delta) = 7pi/6. velocity: 1.0 (not flipped)
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
     * Ensures that the relative encoder optimize method correctly performs angle wraparound
     * (ie. going from pi to 7pi/6 when the target angle is -5pi/6) with optimization.
     * 
     * Current angle: pi/3, target angle: -5pi/6
     * Expected angle: pi/3 - pi/6 (delta) = pi/6. velocity: -1.0 (flipped)
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
     * Ensures that the relative encoder optimize method correctly performs angle wraparound
     * (ie. going from pi to 7pi/6 when the target angle is -5pi/6) without optimization on an
     * angle ouside of [-pi, pi].
     * 
     * Current angle: 5pi/6, target angle: -5pi/6
     * Expected angle: 5pi/6 + pi/3 (delta) = 7pi/6. velocity: 1.0 (not flipped)
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
     * Ensures that the relative encoder optimize method correctly performs angle wraparound
     * (ie. going from pi to 7pi/6 when the target angle is -5pi/6) with optimization on an angle
     * outside of [-pi, pi].
     * 
     * Current angle: pi/3, target angle: -5pi/6
     * Expected angle: pi/3 - pi/6 (delta) = pi/6. velocity: -1.0 (flipped)
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
     * @param currentAngleRads The current angle, in radians.
     * @param targetAngleRads The target angle, in radians.
     * @param expectedAngleRads The expected angle, in radians.
     * @param expectOptimized Whether the state is expected to be optimized (velocity flipped).
     */
    private void runTest(double currentAngleRads, double targetAngleRads, double expectedAngleRads, boolean expectOptimized) {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(targetAngleRads));
        SwerveModuleState optimized = SwerveModule.optimizeWithWraparound(target, new Rotation2d(currentAngleRads));

        assertEquals(expectOptimized ? -1.0 : 1.0, optimized.speedMetersPerSecond);
        assertEquals(expectedAngleRads, optimized.angle.getRadians(), ACCEPTABLE_ANGLE_DELTA);
    }
}
