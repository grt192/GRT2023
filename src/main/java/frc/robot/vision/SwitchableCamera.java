package frc.robot.vision;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * A `VideoSource` that toggles between a front and back camera.
 */
public class SwitchableCamera {
    private final UsbCamera top;
    private final UsbCamera bottom;

    private int TOP_EXPOSURE = 20;
    private int TOP_BRIGHTNESS = 80;

    private int BOTTOM_EXPOSURE = 40;
    private int BOTTOM_BRIGHTNESS = 80;

    private final VideoSink server;
    private final ComplexWidget widget;

    private boolean topActive = true;

    public SwitchableCamera(ShuffleboardTab shuffleboardTab) {
        top = CameraServer.startAutomaticCapture(0);
        bottom = CameraServer.startAutomaticCapture(1);

        top.setResolution(140, 120);
        bottom.setResolution(140, 120);

        top.setFPS(30);
        bottom.setFPS(30);

        // top.setExposureManual(TOP_EXPOSURE);
        // back.setExposureManual(BOTTOM_EXPOSURE);

        // https://github.com/wpilibsuite/allwpilib/blob/main/cscore/src/main/native/linux/UsbCameraImpl.cpp#L108-L124
        // https://www.chiefdelphi.com/t/usb-camera-exposure-too-high-with-setexposuremanual/353630/8
        // top.getProperty("raw_exposure_absolute").set(156);
        // back.getProperty("raw_exposure_absolute").set(156);

        // top.getProperty("raw_brightness").set(40);
        // back.getProperty("raw_exposure_absolute").set(40);

        top.setExposureManual(TOP_EXPOSURE);
        bottom.setExposureManual(BOTTOM_EXPOSURE);
        top.setBrightness(TOP_BRIGHTNESS);
        bottom.setBrightness(BOTTOM_BRIGHTNESS);

        server = CameraServer.getServer();

        widget = shuffleboardTab.add("Intake Camera", getSource())
                .withPosition(8, 3)
                .withSize(4, 2);
    }

    /**
     * Switches the source connected to this camera (toggles between front and
     * back).
     */
    public void switchCamera() {
        setCamera(!this.topActive);
    }

    public void setCamera(boolean useTop) {
        this.topActive = useTop;
        server.setSource(this.topActive ? top : bottom);
        widget.withProperties(Map.of("Rotation", this.topActive ? "NONE" : "QUARTER_CW"));
    }

    /**
     * Gets the `VideoSource` of the camera.
     * 
     * @return The connected `VideoSource`.
     */
    public VideoSource getSource() {
        return server.getSource();
    }
}
