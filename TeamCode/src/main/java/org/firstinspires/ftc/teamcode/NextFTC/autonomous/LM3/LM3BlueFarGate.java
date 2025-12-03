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

@Autonomous(name = "12 Blue Far Side Gate")
public class LM3BlueFarGate extends NextFTCOpMode {
    public LM3BlueFarGate() {
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
    public PathChain gate;
    public PathChain scoreSet2;
    public PathChain grabSet3;
    public PathChain scoreSet3;
    public PathChain prepareHp;
    public PathChain backOutHp;
    public PathChain grabHp;
    public PathChain scoreHp;
    public PathChain park;
    Pose scorePose = new Pose(56,17);

    public void buildPaths() {
        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(60.000, 67.500),
                                new Pose(8.500, 58.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                .build();

        gate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.500, 58.500),
                                new Pose(42.000, 55.000),
                                new Pose(16.000, 67.750)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.000, 67.750),
                                new Pose(59.000, 60.500),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                .build();

        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(54.250, 40.000),
                                new Pose(8.500, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(8.500, 36.000), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                .build();

        prepareHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(8.500, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                .build();

        backOutHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(8.500, 36.000), new Pose(10.000, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(260))
                .build();

        grabHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10.000, 36.000), new Pose(10.000, 11.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(260), Math.toRadians(260))
                .build();

        scoreHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10.000, 11.000), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(260), Math.toRadians(110))
                .build();

        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(119, 70))
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(90))
                .build();
    }


    private Command init_bot() {
        return new SequentialGroup(
                Hoodnf.INSTANCE.setHoodPos(0.34)
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
                //INTAKE ALWAYS ON
                Intakenf.INSTANCE.in(),

                //MAIN SEQUENCE
                new SequentialGroup(

                        //Preloads
                        new ParallelGroup(
                                new FollowPath(scorePreloads, true),

                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1500)
                        ),
                        //Spin up time
                        new Delay(0.4),
                        //ATalign(),
                        transferUpFor(2.2),


                        //SET 2
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet2),
                                        new FollowPath(scoreSet2, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1500)
                        ),
                        new Delay(0.2),
                        transferUpFor(2.6),

                        //SET 3
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet2),
                                        new FollowPath(scoreSet2, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1500)
                        ),
                        new Delay(0.2),
                        transferUpFor(2.6),

                        //SET 4 Human Player
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(prepareHp),
                                        new FollowPath(backOutHp),
                                        new FollowPath(scoreHp, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1500)
                        ),
                        new Delay(0.2),
                        transferUpFor(4)




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