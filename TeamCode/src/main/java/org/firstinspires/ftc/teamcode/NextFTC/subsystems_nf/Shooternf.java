package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Shooternf implements Subsystem {
    public static final Shooternf INSTANCE = new Shooternf();
    private Shooternf() { }

    public MotorEx outtake1, outtake2;
    public MotorGroup shooter;


    private final ControlSystem shooterController = ControlSystem.builder()
            .velPid(0.32,0,0.001)
            .build();

    private boolean enabled = false;

    private enum shooterStates {
        IDLE (0),
        CLOSE_SIDE (-1167),
        FAR_SIDE (-1420);

        private final double shooterState;
        shooterStates(double state) { this.shooterState = state; }
        public double getState() { return shooterState; }
    }

    public Command closeSide() {
        return new RunToVelocity(shooterController, shooterStates.CLOSE_SIDE.getState());
    }
    public Command farSide() {
        return new RunToVelocity(shooterController, shooterStates.FAR_SIDE.getState());
    }
    public Command idle() {
        return new RunToVelocity(shooterController, shooterStates.IDLE.getState());
    }
    public Command setShooterVel(double shooterVel) {
        return new RunToVelocity(shooterController, shooterVel).requires(shooter);
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        shooter.setPower(0); //prevents shooter on in init
    }

    @Override
    public void initialize() {
        outtake1 = new MotorEx("outtake1");
        outtake2 = new MotorEx("outtake2");
        outtake2.reverse();
        shooter = new MotorGroup(outtake1, outtake2);

        disable(); //makes sure motor is off during init
    }

    @Override
    public void periodic() {

        if (!enabled) {
            shooter.setPower(0);
            return;
        }

        shooter.setPower(shooterController.calculate(shooter.getState()));
    }
}