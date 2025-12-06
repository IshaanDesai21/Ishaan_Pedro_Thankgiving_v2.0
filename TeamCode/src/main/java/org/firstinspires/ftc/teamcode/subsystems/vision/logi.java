package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

public class logi {
    AprilTagProcessor apriltagPipeline;

    ballProcessor ballPipeline;

    VisionPortal portal;

    final double ATsize = 6.5;
    final double theta = Math.toRadians(45.2189284116);
    final double[] resolution = new double[]{1920, 1080};

    public logi(HardwareMap hw) {
        apriltagPipeline = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        ballPipeline = new ballProcessor();

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam 1"))
                .addProcessors(apriltagPipeline, ballPipeline)
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                
                .build();

        portal.setProcessorEnabled(apriltagPipeline, true);
        portal.setProcessorEnabled(ballPipeline, false);

    }

    public void enableAT() {
        portal.setProcessorEnabled(apriltagPipeline, true);
    }

    public void enableAT(boolean enabled) {
        portal.setProcessorEnabled(apriltagPipeline, enabled);
    }

    public void enableBall() {
        portal.setProcessorEnabled(ballPipeline, true);
    }

    public void enableBall(boolean enabled) {
        portal.setProcessorEnabled(ballPipeline, enabled);
    }

    public double getATdist() {
        if(!portal.getProcessorEnabled(apriltagPipeline))
            return 0.0;

        for (AprilTagDetection detection : apriltagPipeline.getDetections()) {
            if (detection.id != 20 && detection.id != 24) {
                continue;
            }

            return detection.ftcPose.range;
        }
        return 0.0;
    }

    public double getATangle() {
        if(!portal.getProcessorEnabled(apriltagPipeline))
            return 0.0;

        for (AprilTagDetection detection : apriltagPipeline.getDetections()) {
            if (detection.id != 20 && detection.id != 24) {
                continue;
            }

            return detection.ftcPose.bearing;
        }
        return 0.0;
    }
}

//Not done yet sdadge
class ballProcessor implements VisionProcessor {

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
