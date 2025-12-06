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

@Autonomous(name = "12 Blue Close Gate")
public class LM3BlueCloseGate12 extends NextFTCOpMode {

    public LM3BlueCloseGate12() {
        addComponents(
                new SubsystemComponent(
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    public PathChain scorePreloads;
    public PathChain grabSet2;
    public PathChain hitGate;
    public PathChain scoreSet2;
    public PathChain grabSet3;
    public PathChain scoreSet3;
    public PathChain grabSet4;
    public PathChain scoreSet4;
    public PathChain park;


    private double mx(double x) { return 144 - x; }

    private double mh(double deg) {
        if (deg == 0) return 180;
        if (deg == 180) return 0;
        if (deg == 90 || deg == 270) return deg;
        return 180 - deg;
    }

    public Pose scorePose = new Pose(56, 88);

    public void buildPaths() {

        PedroComponent.follower().setStartingPose(new Pose(mx(126.2), 119, Math.toRadians(144)));


        // ——— PRELOADS ———
        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(mx(126.2), 119),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(mh(44)))
                .build();

        // ——— GRAB SET 2 ———
        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(mx(92.292), 77),
                                new Pose(mx(126.5), 83.4)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(44)), Math.toRadians(mh(0)))
                .build();

        // ——— HIT GATE ———
        hitGate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(mx(126.5), 83.4),
                                new Pose(mx(112), 77),
                                new Pose(mx(130), 71)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(0)), Math.toRadians(mh(90)))
                .build();

        // ——— SCORE SET 2 ———
        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(mx(130), 71),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(90)), Math.toRadians(mh(44)))
                .build();

        // ——— GRAB SET 3 ———
        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(mx(87.760), 55),
                                new Pose(mx(79.313), 57),
                                new Pose(mx(133), 54)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(44)), Math.toRadians(mh(0)))
                .build();

        // ——— SCORE SET 3 ———
        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(mx(133), 54),
                                new Pose(mx(91.262), 56.240),
                                new Pose(mx(95.176), 82.815),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(0)), Math.toRadians(mh(44)))
                .build();

        grabSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(mx(88), 39),
                                new Pose(mx(82), 31),
                                new Pose(mx(132), 35)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(45)), Math.toRadians(mh(0)))
                .build();

        scoreSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mx(132), 35), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(0)), Math.toRadians(mh(45)))
                .build();

        scoreSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mx(132), 35), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(0)), Math.toRadians(mh(45)))
                .build();

        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(mx(88.000), 88.000), new Pose(mx(115.000), 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(mh(44)), Math.toRadians(mh(90)))
                .build();

    }

    private Command init_bot() {
        return new SequentialGroup(
                //higher the number, the
                Hoodnf.INSTANCE.setHoodPos(0.3)
        );
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

                        // PRELOADS
                        new ParallelGroup(
                                new FollowPath(scorePreloads, true),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1255)
                        ),
                        new Delay(0.3),
                        transferUpFor(2.2),

                        // SET 2
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet2),
                                        new FollowPath(hitGate),
                                        new Delay(0.7),
                                        new FollowPath(scoreSet2)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1255)
                        ),
                        new Delay(0.2),
                        transferUpFor(2.5),

                        // SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet3),
                                        new FollowPath(scoreSet3, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1255)
                        ),
                        new Delay(0.2),
                        transferUpFor(2.5),

                        //SET 4
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet4),
                                        new FollowPath(scoreSet4, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1200)
                        ),
                        new Delay(0.2),
                        transferUpFor(2.4),
                        new FollowPath(park)
                        //transferUpFor(2.5)
//                        new ParallelGroup(
//                                transferUpFor(3),
//                                Transfernf.INSTANCE.on()
//                        ),
//                        new FollowPath(park)

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
