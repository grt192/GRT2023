package frc.robot.vision;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * A `VideoSource` that toggles between a front and back camera.
 */
public class SwitchableCamera {
    private final UsbCamera front;
    private final UsbCamera back;

    private final VideoSink server;
    private final ComplexWidget widget;

    private boolean frontActive = true;

    public SwitchableCamera(ShuffleboardTab shuffleboardTab){
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

        
        widget = shuffleboardTab.add("Intake Camera", getSource())
            .withPosition(8, 3)
            .withSize(4, 2)
    }

    /**
     * Switches the source connected to this camera (toggles between front and back).
     */
    public void switchCamera() {
        if (frontActive){
            server.setSource(front);
            widget.withProperties(Map.of("Rotation", "NONE"));
        } else {
            server.setSource(back);
            widget.withProperties(Map.of("Rotation", "QUARTER_CW"));
        }
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
