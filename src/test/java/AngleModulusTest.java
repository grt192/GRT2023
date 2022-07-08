import org.junit.*;
import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.swerve.SwerveModule;

public class AngleModulusTest {
    private static final double ACCEPTABLE_ANGLE_DELTA = 1e-15;

    /**
     * This test ensures that the optimize method correctly handles the case where the
     * target angle and current angle are zero (delta = 0).
     * 
     * Current angle: 0, target angle: 0
     */
    @Test
    public void bothZero() {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(0));
        var optimized = SwerveModule.optimizeModuleState(target, new Rotation2d(0));

        // Velocity: 1.0 (not flipped), angle: 0
        assertEquals(1.0, optimized.getFirst(), 0);
        assertEquals(0, optimized.getSecond(), ACCEPTABLE_ANGLE_DELTA);
    }

    /**
     * This test ensures that the optimize method correctly handles the case where the
     * target angle and current angle are the same (delta = 0).
     * 
     * Current angle: pi/6, target angle: pi/6
     */
    @Test
    public void bothSame() {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(Math.PI / 6.0));
        var optimized = SwerveModule.optimizeModuleState(target, new Rotation2d(Math.PI / 6.0));

        // Velocity: 1.0 (not flipped), angle: pi/6
        assertEquals(1.0, optimized.getFirst(), 0);
        assertEquals(Math.PI / 6.0, optimized.getSecond(), ACCEPTABLE_ANGLE_DELTA);
    }

    /**
     * This test ensures that the optimize method correctly optimizes the module state
     * when the delta is greater than 90 degrees.
     * 
     * Current angle: pi/6, target angle: 5pi/6
     */
    @Test
    public void optimizedNowraparound() {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(5 * Math.PI / 6.0));
        var optimized = SwerveModule.optimizeModuleState(target, new Rotation2d(Math.PI / 6.0));

        // Velocity: -1.0 (flipped), angle: -pi/6
        assertEquals(-1.0, optimized.getFirst(), 0);
        assertEquals(-Math.PI / 6.0, optimized.getSecond(), ACCEPTABLE_ANGLE_DELTA);
    }

    /**
     * This test ensures that the optimize method correctly performs angle wraparound 
     * (ie. going from pi to 7pi/6 when the target angle is -5pi/6) without optimization.
     * 
     * Current angle: 5pi/6, target angle: -5pi/6
     */
    @Test
    public void unoptimizedWraparound() {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(-5 * Math.PI / 6.0));
        var optimized = SwerveModule.optimizeModuleState(target, new Rotation2d(5 * Math.PI / 6.0));

        // Velocity: 1.0 (not flipped), angle: 5pi/6 + pi/3 (delta) = 7pi/6
        assertEquals(1.0, optimized.getFirst(), 0);
        assertEquals(7 * Math.PI / 6.0, optimized.getSecond(), ACCEPTABLE_ANGLE_DELTA);
    }

    /**
     * This test ensures that the optimize method correctly optimizes the module state
     * with angle wraparound (ie. going from pi to 7pi/6 when the target angle is -5pi/6).
     * 
     * Current angle: pi/3, target angle: -5pi/6
     */
    @Test
    public void optimizedWraparound() {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(-5 * Math.PI / 6.0));
        var optimized = SwerveModule.optimizeModuleState(target, new Rotation2d(Math.PI / 3.0));

        // Velocity: -1.0 (flipped), angle: pi/3 - pi/6 (delta) = pi/6
        assertEquals(-1.0, optimized.getFirst(), 0);
        assertEquals(Math.PI / 6.0, optimized.getSecond(), ACCEPTABLE_ANGLE_DELTA);
    }

    /**
     * This test ensures that the optimize method both correctly performs angle wraparound 
     * (ie. going from pi to 7pi/6 when the target angle is -5pi/6) without optimization and 
     * correctly applies the delta to an angle outside of (-pi, pi].
     * 
     * Current angle: 29pi/6, target angle: -5pi/6
     */
    @Test
    public void unoptimizedUnboundedWraparound() {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(-5 * Math.PI / 6.0));
        var optimized = SwerveModule.optimizeModuleState(target, new Rotation2d(29 * Math.PI / 6.0));

        // Velocity: 1.0 (not flipped), angle: 29pi/6 + pi/3 (delta) = 31pi/6
        assertEquals(1.0, optimized.getFirst(), 0);
        assertEquals(31 * Math.PI / 6.0, optimized.getSecond(), ACCEPTABLE_ANGLE_DELTA);
    }

    /**
     * This test ensures that the optimize method both correctly optimizes the module state
     * with angle wraparound (ie. going from pi to 7pi/6 when the target angle is -5pi/6) and 
     * correctly applies the delta to an angle outside of (-pi, pi].
     * 
     * Current angle: 26pi/6, target angle: -5pi/6
     */
    @Test
    public void optimizedUnboundedWraparound() {
        SwerveModuleState target = new SwerveModuleState(1.0, new Rotation2d(-5 * Math.PI / 6.0));
        var optimized = SwerveModule.optimizeModuleState(target, new Rotation2d(26 * Math.PI / 6.0));

        // Velocity: -1.0 (flipped), angle: 26pi/6 - pi/6 (delta) = 25pi/6
        assertEquals(-1.0, optimized.getFirst(), 0);
        assertEquals(25 * Math.PI / 6.0, optimized.getSecond(), ACCEPTABLE_ANGLE_DELTA);
    }
}
