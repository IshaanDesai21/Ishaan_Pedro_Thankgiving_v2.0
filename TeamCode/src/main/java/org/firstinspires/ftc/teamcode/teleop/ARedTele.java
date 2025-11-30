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
public class ARedTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;
    private logi cam;

    private Follower follower;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private boolean autoTurn = false;
    private boolean autoTurnOdo = false;
    private boolean autoTurnVision = false;
    private double goalHeading = 0;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        cam = new logi(hardwareMap);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(126, 118, Math.toRadians(36)));
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
    public void init_loop() {
        telemetryM.debug("This will print your robot's position to telemetry while allowing robot control.");
        telemetryM.update(telemetry);
        follower.update();
    }

    @Override
    public void start() {
        robot.hardwareSoftReset();
        follower.startTeleopDrive(true);
        follower.update();
    }

    @Override
    public void loop() {

        fsm.update();
        follower.update();
        telemetryM.update();
        telemetry.update();

        Pose pose = follower.getPose();
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        double atBearing = Math.toRadians(cam.getATangle());
        double atDistance = cam.getATdist();

        double distance = Math.sqrt(Math.pow((137 - x), 2) + Math.pow((137 - y), 2));


        if (gamepad1.x) {
            follower.setPose(new Pose(x, y, Math.toRadians(0)));
        }

        boolean controllerBusy =
                Math.abs(gamepad1.left_stick_x) > 0.05
                || Math.abs(gamepad1.left_stick_y) > 0.05
                || Math.abs(gamepad1.right_stick_x) > 0.05;

        if (gamepad1.a && !autoTurnOdo) {
            autoTurnOdo = true;
            goalHeading = Math.atan2(137 - y, 137 - x);
        }

        if (gamepad1.dpad_down && !autoTurnVision) {
            autoTurnVision = true;
        }

        double odoHeadingError = angleWrap(goalHeading - heading);
        boolean odoTurnFinished = Math.abs(odoHeadingError) < Math.toRadians(3);

        double atHeadingError = angleWrap(atBearing);
        boolean visionTurnFinished = Math.abs(atHeadingError) < Math.toRadians(0.25);

        if ((autoTurnOdo && odoTurnFinished) || (autoTurnVision && visionTurnFinished) || controllerBusy) {
            autoTurnOdo = false;
            autoTurnVision = false;
        }

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate;

        if (autoTurnOdo) {
            forward = 0;
            strafe = 0;
            double Kp = 1.5; //IF THIS DOESN'T WORK TRY 0.8
            rotate = odoHeadingError * Kp;
            double minPower = 0.11;
            if (Math.abs(rotate) < minPower && Math.abs(odoHeadingError) > Math.toRadians(0.5)) {
                rotate = Math.signum(rotate) * minPower;
            }
        } else if (autoTurnVision) {
            forward = 0;
            strafe = 0;
            double Kp = 0.95;
            rotate = atHeadingError * Kp;
            double minPower = 0.12;
            //prevents robot from going min power when near the target
            if (Math.abs(rotate) < minPower && Math.abs(atHeadingError) > Math.toRadians(0.5)) {
                rotate = Math.signum(rotate) * minPower;
            }
            //follower.holdPoint(new Pose(x, y, pose.getHeading()));
        } else {
            rotate = -gamepad1.right_stick_x;
        }

        follower.setTeleOpDrive(forward, strafe, rotate, true);

        if (gamepad1.startWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (controllerBusy || !follower.isBusy())) {
            //follower.breakFollowing();
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        telemetry.addData("pose", pose);
        telemetry.addData("Heading", heading);
        telemetry.addData("Distance", distance);

        telemetry.addData("AT angle", cam.getATangle());
        telemetry.addData("AT distance", cam.getATdist());

        telemetry.addLine("--------------------------------");
        telemetry.addData("Odo AutoTurn", autoTurnOdo);
        telemetry.addData("Vision AutoTurn", autoTurnVision);
        telemetry.addData("Automated Drive", automatedDrive);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
