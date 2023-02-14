package frc.robot.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;

public class CameraSwitch {
    private UsbCamera front;
    private UsbCamera back;
    
    private int cameraID = 0;
    private VideoSink server;

    public CameraSwitch(){

        front =  CameraServer.startAutomaticCapture(0);
        back =  CameraServer.startAutomaticCapture(1);
        
        server = CameraServer.getServer();
    }

    public void switchCamera(){
        if(cameraID == 0){
            server.setSource(front);
            cameraID = 1;
        }
        else{
            server.setSource(back);
            cameraID = 0;
        }
    }

    public VideoSource getSource(){
        return server.getSource();
    }
}
