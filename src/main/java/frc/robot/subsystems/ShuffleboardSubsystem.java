package frc.robot.subsystems;

import java.text.FieldPosition;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardSubsystem extends SubsystemBase {
    private final ShuffleboardTab tab = Shuffleboard.getTab("testing");
    private final GenericEntry entry = tab.add("Time", 0.0).getEntry();
    private final GenericEntry gentry = tab.add("Offset", 0.0).getEntry();
    private final GenericEntry bentry = tab.add("Toggle", false).getEntry();
    private boolean bool = false;

    private final Field2d cfield = new Field2d();
    private double x = 0;
    private double y = 0;
    private final Rotation2d ratot = Rotation2d.fromDegrees(20);

    // ClassName
    // variableName
    // CONSTANT_NAME

    public ShuffleboardSubsystem() {
        tab.add(new InstantCommand(this::callback, this));
        SmartDashboard.putData("Field", cfield);
        
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
        //cfield.getObject("grobot");
        cfield.setRobotPose(x, y, ratot);
        x += .01;
        y += .01;
        
    }
}
