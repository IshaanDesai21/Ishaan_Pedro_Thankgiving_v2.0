package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

import org.firstinspires.ftc.robotcore.external.Supplier;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

@TeleOp
public class ASlingTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;

    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Pedro Localization
        follower = Constants.createFollower(hardwareMap);

        // Set starting position
        startingPose = new Pose(126, 118, Math.toRadians(36));
        follower.setStartingPose(startingPose);

        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Example path (for automated testing)
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
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
        Pose pose = follower.getPose();

        //----------------------------Telemetry----------------------------\\
        telemetryM.debug("x:" + pose.getX());
        telemetryM.debug("y:" + pose.getY());
        telemetryM.debug("heading:" + Math.toDegrees(pose.getHeading()));
        telemetryM.debug("total heading:" + Math.toDegrees(follower.getTotalHeading()));

        double dist = LocalizationMath.getRedDistance(pose);
        double headingErr = LocalizationMath.getRedHeadingError(pose);

        telemetry.addData("Red Distance (in)", dist);
        telemetry.addData("Red Heading Error (deg)", headingErr);

        //----------------------------Dash Overlay----------------------------\\
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        //----------------------------Drive Controls----------------------------\\
        if (!automatedDrive) {

            // --- ALIGN TO RED GOAL ---
            if (gamepad1.x) { // X button held
                // Create a tiny path to rotate in place
                PathChain alignPath = follower.pathBuilder()
                        .addPath(new Path(new BezierLine(
                                () -> pose,
                                new Pose(pose.getX() + 0.00001, pose.getY()) // almost same position
                        )))
                        .setHeadingInterpolation(
                                HeadingInterpolator.linearFromPoint(
                                        follower::getHeading,
                                        Math.toRadians(pose.getHeading()) + Math.toRadians(headingErr),
                                        1.0
                                )
                        )
                        .build();

                follower.followPath(alignPath);
            } else {
                // Normal driver control
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true // Robot-centric
                );
            }
        }

        // Automated path
        if (gamepad1.startWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        // Stop automated path
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive(true);
            automatedDrive = false;
        }
    }
}
