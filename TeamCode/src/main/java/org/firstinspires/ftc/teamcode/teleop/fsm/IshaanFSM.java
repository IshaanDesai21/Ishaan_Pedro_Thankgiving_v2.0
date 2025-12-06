package org.firstinspires.ftc.teamcode.teleop.fsm;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.vision.PythonLimelight;

public class IshaanFSM {
    // GENERAL ROBOT STATES + CLASSES
    public Robot robot;
    public FSMStates state = FSMStates.BASE_STATE;
    public ControlType type = ControlType.HARDCODED_CONTROL;
    private final GamepadMapping gamepad;

    // SUBSYSTEMS
    private final Intake intake;
    private final Transfer transfer;
    private final Shooter shooter;
    //private final PythonLimelight limelight;

    public IshaanFSM(HardwareMap hardwareMap, GamepadMapping gamepad, Robot robot) {
        this.robot = robot;
        this.gamepad = robot.controls;

        intake = robot.intake;
        transfer = robot.transfer;
        shooter = robot.shooter;
        //limelight = robot.limelight;
    }

    public void update() {
        // Updates driver controls
        gamepad.update();

        switch (state) {
            case BASE_STATE:
                // Always keep shooter & hood in a default position
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
                // Only update shooter/hood dynamically when left bumper is pressed
                if (gamepad.shootFront.locked()) {
                    double x = robot.driver.getPosX(DistanceUnit.INCH);
                    double y = robot.driver.getPosY(DistanceUnit.INCH);

                    double GOAL_X = 142;
                    double GOAL_Y = 142;

                    double dx = GOAL_X - x;
                    double dy = GOAL_Y - y;
                    double distance = Math.hypot(dx, dy);

                    // Distance thresholds
                    double closeDist = 30;
                    double farDist   = 80;

                    double closeVel  = -1100;
                    double farVel    = -1420;
                    double closeHood = 0.42;
                    double farHood   = 0.30;

                    double t = (distance - closeDist) / (farDist - closeDist);
                    t = Math.max(0, Math.min(1, t));

                    double shooterVel  = closeVel + t * (farVel - closeVel);
                    double hoodPos     = closeHood + t * (farHood - closeHood);
                    hoodPos = Math.max(0.05, Math.min(0.7, hoodPos));

                    shooter.setShooterVelocity(shooterVel);
                    shooter.setHoodAngle(hoodPos);

                    boolean shooterReady = Math.abs(shooter.outtake1.getVelocity() - shooterVel) < 40;
                    if (shooterReady) {
                        transfer.transferOn();
                    } else {
                        transfer.hotDog();
                    }
                } else {
                    transfer.hotDog();
                }

                if (!gamepad.shootFront.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.shootBack, gamepad.shootFront, gamepad.transfer);
                }
                break;
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
