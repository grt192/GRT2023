package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {
    private final ShuffleboardTab tab = Shuffleboard.getTab("testing");
    private final GenericEntry entry = tab.add("Time", 0.0).getEntry();
    private final GenericEntry gentry = tab.add("Offset", 0.0).getEntry();
    private final GenericEntry bentry = tab.add("Toggle", false).getEntry();
    private boolean bool = false;

    // ClassName
    // variableName
    // CONSTANT_NAME

    public ShuffleboardSubsystem() {
        tab.add(new InstantCommand(this::callback, this));
    }

    private void callback(){
        bool = !bool;
        System.out.println("hja");
    }

    @Override
    public void periodic() {
        double offset = gentry.getDouble(0.0);
        entry.setValue(Timer.getFPGATimestamp() + offset);
        bentry.setValue(bool);
    }
}
