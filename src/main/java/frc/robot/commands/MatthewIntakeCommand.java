package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsytem;

/**
 * Intakes 1 gamepiece with Pfft Gripper Mech. This command is to be used during autonamous.
 */

public class MatthewIntakeCommand extends CommandBase{
    private final GripperSubsytem gripperSubsystem;
    private final Value state;
    private boolean done;

    public MatthewIntakeCommand (GripperSubsytem gripper){
        this.gripperSubsystem = gripper;
        this.state = gripperSubsystem.getState();
        done = false;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        System.out.print("Gripper is to close");
    }

    @Override
    public void execute() {
        //check if gripper is closed, else close gripper
        if (state == Value.kReverse){
            System.out.println("Gripper is already closed :(");
        }
        else{
            gripperSubsystem.gripToggle();
        }
        done = true;
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("Gripper has closed");
    }

    @Override
    public boolean isFinished() {
        return done;
    }



}