package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.Timer;
=======
>>>>>>> b8c99c4 (updated with PID class and updated constants)
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import edu.wpi.first.wpilibj.Timer;

public class BalancerCommand extends CommandBase {
    private final BaseDrivetrain driveSubsystem;
    private final AHRS ahrs; 
    private final Timer stoptimer;
<<<<<<< HEAD
    private double oldAngle;
    private double currentAngle;

    private double returnPower; //power to be returned to DT
    public boolean reachedStation;
    public boolean passedCenter;

    private final PIDController pid;
=======

    private double returnPower; //power to be returned to DT
    private boolean reachedStation;
    private boolean timerEnabled;
    
    PIDController pid;
>>>>>>> b8c99c4 (updated with PID class and updated constants)

    public BalancerCommand(BaseDrivetrain driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();
<<<<<<< HEAD
        pid = new PIDController(0.3 / 35, 0.0, 0.0); // no deriv successful
        stoptimer = new Timer();
        reachedStation = false;
=======
        pid = new PIDController(0.4/35, 0.05, -0.05);
        stoptimer = new Timer();
>>>>>>> b8c99c4 (updated with PID class and updated constants)
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("---------------------- Balancer initialized ----------------------");
    }

    @Override
    public void execute() {
<<<<<<< HEAD
        // currentAngle = ahrs.getPitch();
        if (!reachedStation) {
            returnPower = 0.80;
            System.out.println(ahrs.getPitch());
            reachedStation = ahrs.getPitch() >= 15.0;
        } else {
            if (!passedCenter) {
                returnPower = 0.15;
                passedCenter = ahrs.getPitch() <= -3.0;
            } else {
                returnPower = -1 * pid.calculate(ahrs.getPitch(), 0);
                System.out.println(returnPower);
                if (Math.abs(ahrs.getPitch()) <= 2.0) {
                    returnPower = 0.0;
                    // if(driveSubsystem instanceof BaseSwerveSubsystem) ((BaseSwerveSubsystem) driveSubsystem).lockNow();
                }
=======
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
>>>>>>> b8c99c4 (updated with PID class and updated constants)
            }
        }
        // else{
        //     returnPower = -1 * pid.calculate(ahrs.getPitch(), 0);
        //     // if((currentAngle - oldAngle) <= -0.5) returnPower = returnPower * ;

        //     if(!timerEnabled && Math.abs(ahrs.getPitch()) <= 2.0){
        //         stoptimer.stop();
        //         stoptimer.reset();
        //         timerEnabled = true;
        //     }
        //     if(Math.abs(ahrs.getPitch()) >= 2.0){
        //         stoptimer.stop();
        //         stoptimer.reset();
        //         timerEnabled = false;
        //     }
        // }

        driveSubsystem.setDrivePowers(returnPower);
        oldAngle = currentAngle;        
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("------------------ Balancing process finished ---------------------");
    }

    @Override
    public boolean isFinished() {
<<<<<<< HEAD
        return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(0.20);
=======
        return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(2.0);
>>>>>>> b8c99c4 (updated with PID class and updated constants)
    }
}
