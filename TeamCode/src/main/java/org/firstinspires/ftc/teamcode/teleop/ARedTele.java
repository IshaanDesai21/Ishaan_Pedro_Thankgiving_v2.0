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
import org.firstinspires.ftc.teamcode.teleop.fsm.IshaanFSM;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class ARedTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;
    private Follower follower;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private boolean autoTurn = false;
    private double goalHeading = 0;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(126, 118, Math.toRadians(36)));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() .addPath(
                        new Path(new BezierLine(
                                follower::getPose, new Pose(follower.getPose().getX() + 0.01, follower.getPose().getY() + 0.01))
                        )) .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading,
                                () -> { double robotX = follower.getPose().getX();
                                    double robotY = follower.getPose().getY();
                                    return Math.atan2(142 - robotY, 142 - robotX); },
                                0.98))
                .build();
    }

    @Override
    public void start() {
        robot.hardwareSoftReset();
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        // Update FSM
        fsm.update();

        // Update follower & telemetry
        follower.update();
        telemetryM.update();

        Pose p = follower.getPose();
        double x = p.getX();
        double y = p.getY();
        double heading = p.getHeading();

        // Reset pose
        if (gamepad1.xWasPressed()) {
            follower.setPose(new Pose(126, 118, Math.toRadians(36)));
        }

        // Heading lock / auto-turn
        if (gamepad1.a && !autoTurn) {
            autoTurn = true;
            goalHeading = Math.atan2(142 - y, 142 - x);
        }

        double headingError = angleWrap(goalHeading - heading);
        boolean turnFinished = Math.abs(headingError) < Math.toRadians(3);
        if (autoTurn && turnFinished) autoTurn = false;

        // Driving inputs
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate;

        if (autoTurn) {
            // Zero forward/strafe during auto-turn
            forward = 0;
            strafe = 0;
            double Kp = 1.0;
            rotate = headingError * Kp;
        } else {
            rotate = -gamepad1.right_stick_x;
        }

        follower.setTeleOpDrive(forward, strafe, rotate, true);

        // Automated path
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Telemetry
        telemetryM.debug("pose", p);
        telemetryM.debug("autoTurn", autoTurn);
        telemetry.addData("Heading", heading);
    }

    private double angleWrap(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
}
