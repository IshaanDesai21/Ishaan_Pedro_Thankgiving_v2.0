package org.firstinspires.ftc.teamcode.subsystems.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.vision.PythonLimelight;
import org.firstinspires.ftc.teamcode.subsystems.vision.logi;

public class Robot {
    // CONFIG
    // right - expansion
    // left - control
    // front - 0
    // back - 1

    // Front transfer 3 control hub
    // Back transfer 5 control hub
    // servo variable hood 0 on control hub
    // outtake1 control hub 3
    // outtake2 control hub 2
    // intake expansion hub port 3

    // MECHANISMS
    public final IMU imu;
    public Intake intake;
    public Transfer transfer;
    public Shooter shooter;
    public Drivetrain drivetrain;
    public GoBildaPinpointDriver driver;

    public GamepadMapping controls;

    public static logi cam;

    //public PythonLimelight limelight;

    public Robot(HardwareMap hardwareMap, GamepadMapping controls) {
        this.controls = controls;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        cam = new logi(hardwareMap);

        //ISHAAN TOOK THIS OUT SO THE REVERSING MOTORS DOES NOT CLASH WITH THE RR MOTORS
        drivetrain = new Drivetrain(hardwareMap, imu, controls);

        //limelight = new PythonLimelight(hardwareMap);
    }

    public void hardwareSoftReset() {
        transfer.transferOff();
        shooter.hoodToBack();
        intake.intakeOff();
        shooter.setShooterVelocity(0);
    }
}
