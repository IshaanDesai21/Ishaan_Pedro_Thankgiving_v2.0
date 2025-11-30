package org.firstinspires.ftc.teamcode.teleop.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.PythonLimelight;

@TeleOp
public class LimelightTest extends OpMode {

    public PythonLimelight limelight;

    @Override
    public void init() {
        limelight = new PythonLimelight(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        telemetry.addData("limelight angle", Math.toDegrees(limelight.getAngle()));
        telemetry.addData("limelight dist", Math.toDegrees(limelight.getDistance()));
        telemetry.addData("limelight nav", (limelight.getLastNav()));
        telemetry.addData("limelight obelisk", (limelight.getObelisk().order));
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        limelight.close();
    }
}
