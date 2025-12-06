package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.vision.logi;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class AVisionTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;
    private logi cam;

    private Follower follower;
    private boolean autoTurnVision = false;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    public static Pose pose;


    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        cam = new logi(hardwareMap);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(new Pose(18, 118, Math.toRadians(144)));
        pose = follower.getPose();
        follower.setStartingPose(pose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        follower::getPose,
                        new Pose(105.4, 33.4)
                )))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, Math.toRadians(90), 0.9)
                )
                .build();
    }

    @Override
    public void start() {
        robot.hardwareSoftReset();
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

        fsm.update();
        follower.update();
        telemetryM.update();
        telemetry.update();

        Pose pose = follower.getPose();
        double heading = pose.getHeading();

        double atBearing = Math.toRadians(cam.getATangle());
        double atHeadingError = angleWrap(atBearing);
        boolean visionTurnFinished = Math.abs(atHeadingError) < Math.toRadians(0.2);

        boolean controllerBusy =
                Math.abs( gamepad1.left_stick_x) > 0.05
            || Math.abs(gamepad1.left_stick_y) > 0.05
            || Math.abs(gamepad1.right_stick_x) > 0.05;

        if (gamepad1.xWasPressed()) {
            follower.setPose(new Pose(pose.getX(), pose.getY(), Math.toRadians(90)));
        }

        if (gamepad1.dpad_down && !autoTurnVision) {
            autoTurnVision = true;
        }

        if ((autoTurnVision && visionTurnFinished) || controllerBusy) {
            autoTurnVision = false;
        }

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate;

        if (autoTurnVision) {
            forward = 0;
            strafe = 0;

            double Kp = 0.95;
            rotate = atHeadingError * Kp;

            double minPower = 0.12;
            if (Math.abs(rotate) < minPower && Math.abs(atHeadingError) > Math.toRadians(0.5)) {
                rotate = Math.signum(rotate) * minPower;
            }

        } else {
            rotate = -gamepad1.right_stick_x;
        }

        follower.setTeleOpDrive(forward, strafe, rotate, true);

        if (gamepad1.startWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (controllerBusy || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        telemetry.addData("pose", pose);
        telemetry.addData("Heading", heading);
        telemetry.addData("AT angle", cam.getATangle());
        telemetry.addData("AT dist", cam.getATdist());
        telemetry.addLine("--------------------------------");
        telemetry.addData("Vision AutoTurn", autoTurnVision);
        telemetry.addData("AutoPark", automatedDrive);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
