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

import java.net.PasswordAuthentication;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "12 Red Far Side")
public class LM3RedFarSide12 extends NextFTCOpMode {
    public LM3RedFarSide12() {
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
    public PathChain grabSet1;
    public PathChain scoreSet1;
    public PathChain grabSet2;
    public PathChain scoreSet2;
    public PathChain prepareHp;
    public PathChain grabHp;
    public PathChain scoreHp;
    public PathChain grabSet4;
    public PathChain scoreSet4;
    public PathChain park;
    Pose scorePose = new Pose(88,17);

    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(88, 8.2, Math.toRadians(90)));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.000, 8.200), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                .build();

        grabSet1 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(88.500, 41.000),
                                new Pose(132.500, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                .build();

        scoreSet1 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132.500, 36.000), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(97.000, 38.000),
                                new Pose(132, 35)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132, 35), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                .build();

        prepareHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(134.000, 33.250))
                )
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(279))
                .build();

        grabHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 33.250), new Pose(134.000, 10.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(280), Math.toRadians(280))
                .build();

        scoreHp = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 10.400), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(280), Math.toRadians(70))
                .build();

        grabSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(80.500, 65.000),
                                new Pose(137.5, 59.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                .build();

        scoreSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(137.5, 59.500), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))
                .build();

        park = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(25.000, 70.000))
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
                        transferUpFor(2.5),

                        //SET 3 Human Player
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(prepareHp),
                                        new FollowPath(grabHp),
                                        new FollowPath(scoreHp, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1500)
                        ),
                        transferUpFor(2.5),

                        //SET 4
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(grabSet4),
                                        new FollowPath(scoreSet4, true)
                                ),
                                baseState(),
                                Shooternf.INSTANCE.setShooterVel(-1500)
                        ),
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