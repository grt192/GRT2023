package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.drivetrain.BaseDrivetrain;
import frc.robot.subsystems.drivetrain.BaseSwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class BalancerCommand extends CommandBase {
    private final BaseSwerveSubsystem driveSubsystem;
    private final AHRS ahrs; 
    private final Timer stoptimer;
    private double oldAngle;
    private double currentAngle;

    private double returnPower; //power to be returned to DT
    public boolean reachedStation;
    private boolean passedCenter;
    private boolean timerEnabled;
    
    PIDController pid;

    public BalancerCommand(BaseSwerveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.ahrs = driveSubsystem.getAhrs();
        pid = new PIDController(0.35/35, 0.0, 0.0);
        stoptimer = new Timer();
        reachedStation = false;
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
        // currentAngle = ahrs.getPitch();
        if(!reachedStation) {
            returnPower = 0.80;
            System.out.println(ahrs.getPitch());
            if(ahrs.getPitch() >= 15.0) reachedStation = true;
        }
        else{
            if(!passedCenter){
                returnPower = 0.3;
                if(ahrs.getPitch() <= -5.0) passedCenter = true;
            }
            else{
                returnPower = -0.25;
                if(Math.abs(ahrs.getPitch()) <= 2.0){
                    returnPower = 0.0;
                    driveSubsystem.lockNow();
                }
                
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

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("------------------ Balancing process finished ---------------------");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return reachedStation && Math.abs(ahrs.getPitch()) <= 2.0 && stoptimer.hasElapsed(0.20);
    }
}
