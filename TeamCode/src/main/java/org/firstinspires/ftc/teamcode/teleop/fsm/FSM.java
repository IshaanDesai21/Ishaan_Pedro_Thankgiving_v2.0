package org.firstinspires.ftc.teamcode.teleop.fsm;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.vision.PythonLimelight;

public class FSM {
    // GENERAL ROBOT STATES + CLASSES
    public Robot robot;
    public FSMStates state = FSMStates.BASE_STATE;
    public ControlType type = ControlType.HARDCODED_CONTROL;
    private final GamepadMapping gamepad;


    // SUBSYSTEMS
    private final Intake intake;
    //private Turret turret;
    private final Transfer transfer;
    private final Shooter shooter;
    //private final Drivetrain drivetrain;

    //private final PythonLimelight limelight;

    public FSM(HardwareMap hardwareMap, GamepadMapping gamepad, Robot robot) {
        this.robot = robot;
        this.gamepad = robot.controls;

        intake = robot.intake;
        //turret = robot.turret;
        transfer = robot.transfer;

        shooter = robot.shooter;

//        drivetrain = robot.drivetrain;
        //limelight = robot.limelight;
    }

    public void update() {
        // Updates driver controls here as well

        // Updates all other controls
        gamepad.update();

        // drive.update();
        // pose = drive.getPose();
        // TODO - Get robot pos from localization

        switch (state) {
            case BASE_STATE:
                // TODO: still keep always running depending on spin up time
                shooter.shootFromFront();
                shooter.hoodToFront();

                intake.intakeOn();
                transfer.hotDog();

                if (gamepad.outtake.locked()) {
                    state = FSMStates.OUTTAKING;
                }

                if (gamepad.shootBack.locked()) {
                    state = FSMStates.SHOOT_BACK;
                }

                if (gamepad.shootFront.locked()) {
                    state = FSMStates.SHOOT_FRONT;
                }



                break;
            case OUTTAKING:
                intake.intakeReverse();
                if (!gamepad.outtake.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer, gamepad.shootFront, gamepad.shootBack);
                }
                break;

            case SHOOT_BACK:
                shooter.shootFromBack();
                shooter.hoodToBack();

                if (shooter.outtake1.getVelocity() <= Shooter.outtakeVels.HARDCODED_SHOOT_BACK.getOuttakeVel() + 50) {
                    transfer.transferOn();
                }

                if (!gamepad.shootBack.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.shootBack, gamepad.shootFront, gamepad.transfer);
                }
                break;
            case SHOOT_FRONT:
                shooter.shootFromFront();

                transfer.transferOn();

                if (!gamepad.shootFront.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.shootBack, gamepad.shootFront, gamepad.transfer);
                }
                break;

//            case TURRET:
                // intake on
                // TODO: FIX THIS FOR TURRET
                //turret.setTurretPos(turret.calcTurretVal(pose.getX(), pose.getY(), pose.getX(), pose.getY(), pose.getHeading()), 1);
        }
    }

    public enum FSMStates {
        BASE_STATE,
        SHOOT_FRONT,
        SHOOT_BACK,
        OUTTAKING,
        TRANSFER
    }

    public enum ControlType {
        HARDCODED_CONTROL,
        PID_CONTROL
    }

}
