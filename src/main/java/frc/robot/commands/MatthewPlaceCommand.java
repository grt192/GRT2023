package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsytem;

public class MatthewPlaceCommand extends CommandBase{
    private final GripperSubsytem gripperSubsystem;
    private final Value state;
    private boolean done;

    public MatthewPlaceCommand (GripperSubsytem gripper){
        this.gripperSubsystem = gripper;
        this.state = gripperSubsystem.getState();
        done = false;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        System.out.print("Gripper is to open");
    }

    @Override
    public void execute() {
        if (state == Value.kForward){
            System.out.println("Gripper is already open :(");
        }
        else{
            gripperSubsystem.gripToggle();
        }
        done = true;
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("Gripper has opened");
    }

    @Override
    public boolean isFinished() {
        return done;
    }



}