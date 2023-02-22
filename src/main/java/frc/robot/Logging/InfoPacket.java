package frc.robot.Logging;

import edu.wpi.first.math.geometry.Pose2d;

public class InfoPacket implements Message{
    
    private Pose2d position;
    private double velocity;
    private String m;

    InfoPacket(String m, Pose2d position, double velocity){
        this.position = position;
        this.velocity = velocity;
        this.m = m;

    }

    @Override
    public String display(){
        return "[INFO] Message: " + m +  " Position: (" + position.getX() + ", " + position.getY() + ") Heading: " + position.getRotation().getDegrees();
    }

}

