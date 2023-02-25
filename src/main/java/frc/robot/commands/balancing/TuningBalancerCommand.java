package frc.robot.commands.balancing;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.BaseDrivetrain;

public class TuningBalancerCommand extends DefaultBalancerCommand {


    ArrayList<Double> time = new ArrayList<Double>();
    ArrayList<Double> pitch = new ArrayList<Double>();
    ArrayList<Double> power = new ArrayList<Double>();

    Timer timer;

    public TuningBalancerCommand(BaseDrivetrain driveSubsystem) {
        super(driveSubsystem);
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute(){
        super.execute();
        pitch.add(super.getPitch());
        power.add(super.getPower());
        time.add(timer.get());
    }

    @Override
    public boolean isFinished(){
        if(super.isFinished()){
            System.out.println("TIME: " + time);
            System.out.println(pitch);
            System.out.println(power);
            return true;
        }
        else{
            return false;
        }
    }
}
