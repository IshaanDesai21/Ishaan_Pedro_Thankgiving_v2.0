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
public class ARedTele extends OpMode {

    /** @noinspection FieldCanBeLocal*/
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

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38.7, 33.4))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        //TODO - Edit endT Threshold
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
    //Updates fsm, follower, and telemetry systems
        fsm.update();
        follower.update();
        telemetryM.update();
        telemetry.update();


    //Robot pose from localization ---------------------------------------------------
        Pose pose = follower.getPose();
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

    //Relocalizes robot --------------------------------------------------------------
        if (gamepad1.xWasPressed()) {
            follower.setPose(new Pose(126, 118, Math.toRadians(36)));
        }

    // Heading lock -------------------------------------------------------------------

        // Auto-turn if not already turning if "a" is clicked
        if (gamepad1.a && !autoTurn) {
            autoTurn = true;
            goalHeading = Math.atan2(144 - y, 144 - x);
        }

        //gets heading error and angle wraps it.
        double headingError = angleWrap(goalHeading - heading);
        //checks if turn is finished (2 degree threshold)
        boolean turnFinished = Math.abs(headingError) < Math.toRadians(2);
        //turns turn mode off if its in turn mode and the turn is finished
        if (autoTurn && turnFinished) {
            //HOLDS THE POINT WHEN SHOOTING
            follower.holdPoint(follower.getPose());
            autoTurn = false;
        }

    // Driving inputs -------------------------------------------------------------
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate;

        if (autoTurn) {
            //Makes sure robot cannot go straight or strafe when turning
            forward = 0;
            strafe = 0;
            //Proportional gain for turn
            //TODO - tune this
            double Kp = 0.8;
            rotate = headingError * Kp;
        } else {
            //if auto turn isnt deciding the turn rate, then it just gets it fom gamepad for regular driving
            rotate = -gamepad1.right_stick_x;
        }
        //Sets Drive to have driver inputs
        follower.setTeleOpDrive(forward, strafe, rotate, true);


    // Auto park -------------------------------------------------------------------
        if (gamepad1.startWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Checks if path in progress, or if "b", the cancel button is pressed
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

    //Telemetry -------------------------------------------------------------------
        telemetry.addData("pose", pose);
        telemetry.addData("Heading", heading);
        telemetry.addLine("--------------------------------");
        telemetry.addData("autoTurn", autoTurn);
        telemetry.addData("Automated Drive", automatedDrive);

    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2*Math.PI;
        while (angle < -Math.PI) angle += 2*Math.PI;
        return angle;
    }
}
