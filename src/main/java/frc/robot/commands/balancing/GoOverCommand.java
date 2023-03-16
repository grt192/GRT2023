package frc.robot.commands.balancing;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;

public class GoOverCommand extends CommandBase {
    private final BaseDrivetrain driveSubsystem;
    private final AHRS ahrs; 

    private double returnDrivePower; // drive power to be returned to DT
    private double returnAngularPower; // angular power to return to DT (for heading correction)

    private boolean reachedStation;
    private boolean passedCenter;
    private boolean overStation;
    private boolean waiting;

    private final Timer timer;

    public GoOverCommand(BaseDrivetrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();

        timer = new Timer();
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
        if (!reachedStation) {
            returnDrivePower = -0.80;
            reachedStation = ahrs.getPitch() <= -15.0;
        }

        if (reachedStation && !passedCenter) {
            returnDrivePower = 0.4;
            if (ahrs.getPitch() >= -0.0){
                returnDrivePower = 0.4;
                passedCenter = true;
            } 
        }
        if (reachedStation && passedCenter && !overStation) {
            if (Math.abs(ahrs.getPitch()) <= 0.5) {
                overStation = true;
            }
            else returnDrivePower = 0.4;
        }

        if (overStation && !waiting) {
            timer.reset();
            timer.start();
            returnDrivePower = 0.4;
            waiting = true;
        }

        driveSubsystem.setDrivePowers(returnDrivePower);
    }

    @Override 
    public void end(boolean interrupted){
        driveSubsystem.setDrivePowers(0.0);
    }

    @Override
    public boolean isFinished() {
        return overStation && timer.hasElapsed(0.25);
    }
}
