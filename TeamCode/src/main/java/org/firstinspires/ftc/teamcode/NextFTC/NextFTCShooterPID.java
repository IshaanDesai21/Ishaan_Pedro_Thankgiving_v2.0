package org.firstinspires.ftc.teamcode.NextFTC;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@Config
@TeleOp
public class NextFTCShooterPID extends OpMode {
    MotorEx outtake1, outtake2;
    MotorGroup shooter;
    private ControlSystem shooterController;


    public static double p, i, d;
    public static double kV, kA, kS;
    public static int shooterVel = -1100;

    @Override
    public void init() {
        outtake1 = new MotorEx("outtake1");
        outtake2 = new MotorEx("outtake2");
        outtake2.reverse();
        shooter = new MotorGroup(outtake1, outtake2);

        shooterController = ControlSystem.builder()
                .posPid(new PIDCoefficients(p, i, d))
                .basicFF(new BasicFeedforwardParameters(kV, kA, kS))
                .build();


    }

    @Override
    public void loop() {
        shooterController.setGoal(new KineticState(0.0, shooterVel));

        //shooter.setPower(shooterController.calculate(shooter.getState()));
    }
}