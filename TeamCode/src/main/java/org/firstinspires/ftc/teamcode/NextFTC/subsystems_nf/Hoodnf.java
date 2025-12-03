package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Hoodnf implements Subsystem {
    public static final Hoodnf INSTANCE = new Hoodnf();
    private Hoodnf() {}

    private final ServoEx variableHood = new ServoEx("variableHood");

    public Command closeSide() {
        return new SetPosition(variableHood, 0.2).requires(this);
    }
    public Command closeLastSet() {
        return new SetPosition(variableHood, 0.61).requires(this);
    }

    public Command farSide() {
        return new SetPosition(variableHood, 0.42).requires(this);
    }

    public Command setHoodPos(double hoodPosition) {
        return new SetPosition(variableHood, hoodPosition).requires(this);
    }

}