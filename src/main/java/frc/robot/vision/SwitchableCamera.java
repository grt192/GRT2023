package frc.robot.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;

/**
 * A `VideoSource` that toggles between a front and back camera.
 */
public class SwitchableCamera {
    private final UsbCamera front;
    private final UsbCamera back;

    private final VideoSink server;

    private boolean frontActive = true;

    public SwitchableCamera(){
        front = CameraServer.startAutomaticCapture(0);
        back = CameraServer.startAutomaticCapture(1);

        front.setResolution(140, 120);
        back.setResolution(140, 120);

        front.setFPS(30);
        back.setFPS(30);

        front.setExposureManual(40);
        back.setExposureManual(40);

        front.setBrightness(50);
        back.setBrightness(50);

        server = CameraServer.getServer();
    }

    /**
     * Switches the source connected to this camera (toggles between front and back).
     */
    public void switchCamera() {

        server.setSource(frontActive ? front : back);
        frontActive = !frontActive;
    }

    /**
     * Gets the `VideoSource` of the camera.
     * @return The connected `VideoSource`.
     */
    public VideoSource getSource() {
        return server.getSource();
    }
}
