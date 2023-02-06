package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import edu.wpi.first.wpilibj.Timer;

public class BalancerCommand extends CommandBase {
    private final BaseDrivetrain driveSubsystem;
    private final AHRS ahrs; 
    private final Timer stoptimer;

    private double returnPower; //power to be returned to DT
    private boolean reachedStation;
    private boolean timerEnabled;
    
    PIDController pid;

    public BalancerCommand(BaseDrivetrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();
        pid = new PIDController(0.4/35, 0.05, -0.05);
        stoptimer = new Timer();
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("---------------------- Balancer initialized ----------------------");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!reachedStation) {
            returnPower = 0.90;
            System.out.println(ahrs.getPitch());
            if(ahrs.getPitch() >= 15.0) reachedStation = true;
        }
        else{
            returnPower = -1 * pid.calculate(ahrs.getPitch(), 0);
            // returnPower  = 1.8 * 0.01142857143 * ahrs.getPitch(); // MAX_ERR * Kp = MAX_PWR   35 * Kp = 0.4  Kp = 0.4 / 35
            if(!timerEnabled && Math.abs(ahrs.getPitch()) <= 2.0){
                stoptimer.stop();
                stoptimer.reset();
                timerEnabled = true;
            }
            if(Math.abs(ahrs.getPitch()) >= 2.0){
                stoptimer.stop();
                stoptimer.reset();
                timerEnabled = false;
            }
        }

        driveSubsystem.setDrivePowers(returnPower);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("------------------ Balancing process finished ---------------------");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(2.0);
    }
}
