package frc.robot.commands.balancing;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;

/**
 * A command that moves robot over charging station at a varying speed, 
 *  changing speed based on the stage of the charging station the robot is on. 
 * 
 * Command enables the robot to go over the charging station.
 * Mobility outside of the 'community' would earn 3 extra points in the 2023 game.
 * 
 * This command was not used at any 2023 competition. 
 * Testing on this command was not fully complete, so the overall functionality of this code is unknown. 
 */
public class GoOverCommand extends CommandBase {
    private static final double APPROACH_STATION_POWER = -0.75;
    private static final double CLIMBING_STATION_POWER = -0.45;
    private static final double PAST_CENTER_POWER = -0.25;
    private static final double POWER_SCALE = 0.5;

    private final BaseDrivetrain driveSubsystem;
    private final AHRS ahrs; 

    private double returnDrivePower;

    private boolean reachedStation;
    private boolean passedCenter;
    private boolean overStation;
    private boolean waiting;

    private final Timer passedCenterTimer;
    private final Timer waitTimer;

    public GoOverCommand(BaseDrivetrain driveSubsystem, boolean isRed) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();
        waitTimer = new Timer();
        passedCenterTimer = new Timer();
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        reachedStation = false;
        passedCenter = false;
        overStation = false;
        waiting = false;
    }

    @Override
    public void execute() {
        double currentPitch = ahrs.getPitch();
        System.out.println("Pitch" + currentPitch);

        if (!reachedStation) {
            returnDrivePower = APPROACH_STATION_POWER;
            reachedStation = currentPitch <= -7.0;
            if (reachedStation) System.out.println("reached station");
        } else if (!passedCenter) {
            returnDrivePower = CLIMBING_STATION_POWER;
            passedCenter = currentPitch >= -7.0;
            if (passedCenter) {
                System.out.println("passed center");
                passedCenterTimer.start();
            }
        } else if (!overStation) {
            returnDrivePower = PAST_CENTER_POWER;
            if (passedCenterTimer.hasElapsed(0.35)) { // wait a little bit before allowing ourselves to think we're passed center (bc otherwise gaslit by chargng station)
                overStation = Math.abs(currentPitch) <= 2.0;
                if (overStation) System.out.println("over station");
            }
        } else if (!waiting) {
            waitTimer.start();
            returnDrivePower = -0.2;
            waiting = true;
            System.out.println("waiting");
        }

        if (driveSubsystem instanceof BaseSwerveSubsystem) {
            ((BaseSwerveSubsystem) driveSubsystem).setDrivePowersWithHeadingLock(
                returnDrivePower * POWER_SCALE,
                0.0,
                new Rotation2d(Math.PI),
                true
            );
        } else {
            driveSubsystem.setDrivePowers(returnDrivePower * POWER_SCALE);
        }
    }

    @Override 
    public void end(boolean interrupted) {
        driveSubsystem.setDrivePowers(0.0);
        System.out.println("ended");

        waitTimer.stop();
        waitTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return waitTimer.hasElapsed(1.5);
    }
}
