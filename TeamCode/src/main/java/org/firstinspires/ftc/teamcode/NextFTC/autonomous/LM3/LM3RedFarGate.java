package org.firstinspires.ftc.teamcode.NextFTC.autonomous.LM3;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "12 Red Far Side Gate")
public class LM3RedFarGate extends NextFTCOpMode {

    public LM3RedFarGate() {
        addComponents(
                new SubsystemComponent(
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads, grabSet2, scoreSet2, grabSet3, scoreSet3, prepareHp, grabHp, scoreHp, park;

    public PathChain gate;

    private double mx(double x) { return 144 - x; }

    private double mh(double deg) {
        if (deg == 0) return 180;
        if (deg == 180) return 0;
        if (deg == 90 || deg == 268) return deg;
        return 180 - deg;
    }

    Pose scorePose = new Pose(88, 17);

    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(88, 8.2, Math.toRadians(90)));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(88, 8), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(84, 65), new Pose(132, 57)))
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();


        gate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132, 57.000),
                                new Pose(89, 66.000),
                                new Pose(130, 70.400)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(new Pose(130, 70.4), new Pose(85, 60.5), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
                .build();

        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(87, 36), new Pose(134, 32.8)))
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(134, 32.8), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();


        prepareHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(132, 50)))
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(300))
                .build();

        grabHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(132, 50), new Pose(132, 11)))
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(290))
                .build();

        scoreHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(new Pose(134, 11), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(68))
                .build();

        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(132, 68)))
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(90))
                .build();
    }

    private Command init_bot() {
        return Hoodnf.INSTANCE.setHoodPos(0.34);
    }

    private Command transferUpFor(double time) {
        return new ParallelGroup(
                Transfernf.INSTANCE.on(),
                new Delay(time)
        );
    }

    private Command baseState() {
        return new ParallelGroup(
                Transfernf.INSTANCE.hotdog(),
                Hoodnf.INSTANCE.setHoodPos(0.3)
        );
    }

    private Command autonomous() {
        return new ParallelGroup(
                Intakenf.INSTANCE.in(),
                new SequentialGroup(
                        // Preloads
                        new ParallelGroup(
                                new FollowPath(scorePreloads, true),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        ),
                        new Delay(0.3),
                        transferUpFor(2.2),

                        // SET 2
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet2),
                                        new FollowPath(gate),
                                        new FollowPath(scoreSet2, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        ),
                        new Delay(0.2),
                        transferUpFor(2.6),

                        // SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet3),
                                        new FollowPath(scoreSet3)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        ),
                        new Delay(0.2),
                        transferUpFor(2.6),

                        // Human Player
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(prepareHp),
                                        //new Delay(0.3),
                                        new FollowPath(grabHp),
                                        new FollowPath(scoreHp, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1515)
                        ),
                        new Delay(0.3),
                        transferUpFor(2.4),
                        new FollowPath(park)
                )
        );
    }

    @Override
    public void onInit() {
        buildPaths();
        init_bot().schedule();
        Shooternf.INSTANCE.disable();
    }

    @Override
    public void onStartButtonPressed() {
        autonomous().schedule();
        Shooternf.INSTANCE.enable();
    }
}
