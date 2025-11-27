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
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class ABlueTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;
    private Follower follower;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private boolean autoTurn = false;
    private double goalHeading = 0;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(-126, 118, Math.toRadians(144)));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        follower::getPose,
                                        new Pose(
                                                follower.getPose().getX() + 0.1,
                                                follower.getPose().getY() + 0.1
                                        )
                                )
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                () -> {
                                    double x = follower.getPose().getX();
                                    double y = follower.getPose().getY();
                                    return Math.atan2(144 - y, -144 - x);
                                },
                                0.98
                        )
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

        Pose p = follower.getPose();
        double x = p.getX();
        double y = p.getY();
        double heading = p.getHeading();

        if (gamepad1.a && !autoTurn) {
            autoTurn = true;
            double GX = -144;
            double GY = 144;
            goalHeading = Math.atan2(GY - y, GX - x);
        }

        double headingError = angleWrap(goalHeading - heading);
        boolean finished = Math.abs(headingError) < Math.toRadians(3);

        if (autoTurn && finished) autoTurn = false;

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate;

        if (slowMode) {
            forward *= slowModeMultiplier;
            strafe *= slowModeMultiplier;
        }

        if (autoTurn) rotate = headingError * 1.8;
        else {
            rotate = -gamepad1.right_stick_x;
            if (slowMode) rotate *= slowModeMultiplier;
        }

        follower.setTeleOpDrive(forward, strafe, rotate, true);

        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.rightBumperWasPressed()) slowMode = !slowMode;
        if (gamepad1.xWasPressed()) slowModeMultiplier += 0.25;
        if (gamepad2.yWasPressed()) slowModeMultiplier -= 0.25;

        telemetryM.debug("pose", p);
        telemetryM.debug("autoTurn", autoTurn);
    }

    private double angleWrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
