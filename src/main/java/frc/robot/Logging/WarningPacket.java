package frc.robot.Logging;

import edu.wpi.first.math.geometry.Pose2d;

public class WarningPacket implements Message{
    
    private double time;
    private Pose2d position;
    private double velocity;
    private String m;

    WarningPacket(String m, double time, Pose2d position, double velocity){
        this.time = time;
        this.position = position;
        this.velocity = velocity;
        this.m = m;

    }

    @Override
    public String display(){
        return "[WARN] Message: " + m +  "Time: " + time + " Position: (" + position.getX() + ", " + position.getY() + ") Heading: " + position.getRotation().getDegrees();
    }

}

