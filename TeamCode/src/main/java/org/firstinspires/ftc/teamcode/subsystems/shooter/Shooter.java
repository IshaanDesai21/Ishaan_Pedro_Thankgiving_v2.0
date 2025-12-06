package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.ARedTele;

public class Shooter {
    public final DcMotorEx outtake1;
    public final DcMotorEx outtake2;
    public final Servo variableHood;

    public Shooter(HardwareMap hardwareMap) {
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake1.setVelocityPIDFCoefficients(578,0,0,70);
        outtake2.setVelocityPIDFCoefficients(578,0,0,70);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        variableHood = hardwareMap.get(Servo.class, "variableHood");
    }

    public enum outtakeVels {
        PID_SHOOT(shootVel),
        // 5.059
        HARDCODED_SHOOT_FRONT(-1120),
        // 5.954
        HARDCODED_SHOOT_BACK(-1400),
        IDLE(0);

        private final double outtake_vels;

        outtakeVels(double pos) {
            this.outtake_vels = pos;
        }
        public double getOuttakeVel() {
            return outtake_vels;
        }
    }

    //-----------------Math-----------------\\
    private static final double g = 9.79284;
    // y distance to goal - to the middle of the shooter
    private static final double H = 0.360837;
    private static double shootVel;

    // calculates target velocity depending on the distance the robot is from the goal
    public double calculateShooterVel() {
        double R = Robot.cam.getATdist();
        shootVel = Math.sqrt(H * g + g * Math.sqrt(Math.pow(R, 2) + Math.pow(H, 2)));
        return convertMPSToRPM(shootVel);
    }

    // converts the target velocity from meters per second to rpm for DcMotor
    public static double convertMPSToRPM(double mpsVel) {
        double c = -2.28611;
        double a = 12.79622;
        double b = 0.000790302;
        double lnArgument = 1.0 - ((mpsVel - c)/a);
        return -Math.log(lnArgument)/b; // Math.log is natural logarithm
    }

    // HOOD ANGLE CALCULATIONS
    // ---------------------------------
    private static double hoodAngle;

    // returns the target angle in degrees depending on our distance from the april tag
    public double calculateHoodAngle() {
        double R = Robot.cam.getATdist();
        hoodAngle = Math.atan(Math.pow(calculateShooterVel(), 2)/(g * R));
        return Math.toRadians(hoodAngle);
    }

    // converts the target angle from calculateHoodAngle() to a servo position from 0-1
    public double convertTargetAngleToHoodPos(double targetAngle) {
        double m = 42.8718;
        double b = 37.09643;
        return (targetAngle - b)/m;
    }

    public static double getShootVel() {
        return shootVel;
    }


//-------------------------------------------------------------------------------

    public void setShooterVelocity(double velo) {
        outtake1.setVelocity(velo);
        outtake2.setVelocity(velo);
    }

    public void setShooterPower(double power) {
        outtake1.setPower(power);
        outtake2.setPower(power);
    }

    public void setHoodAngle(double angle) {
        variableHood.setPosition(angle);
    }
    // fully down is .6
    // fully up is .1
    public void hoodToBack() {
        variableHood.setPosition(.175);
    }

    public void hoodToFront() {
        variableHood.setPosition(.5);
    }

    public void shootFromBack() {
        outtake1.setVelocity(outtakeVels.HARDCODED_SHOOT_BACK.getOuttakeVel());
        outtake2.setVelocity(outtakeVels.HARDCODED_SHOOT_BACK.getOuttakeVel());
    }

    public void shootFromFront() {
        outtake1.setVelocity(outtakeVels.HARDCODED_SHOOT_FRONT.getOuttakeVel());
        outtake2.setVelocity(outtakeVels.HARDCODED_SHOOT_FRONT.getOuttakeVel());
    }

}



