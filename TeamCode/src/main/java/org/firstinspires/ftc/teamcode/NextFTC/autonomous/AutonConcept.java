package org.firstinspires.ftc.teamcode.NextFTC.autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.AutonConceptSequences;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous(name = "CONCEPT Red Close Pedro")
public class AutonConcept extends NextFTCOpMode {
    public AutonConcept() {
        addComponents(
                new SubsystemComponent(
                        AutonConceptSequences.INSTANCE,
                        Intakenf.INSTANCE, Transfernf.INSTANCE,
                        Shooternf.INSTANCE, Hoodnf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );

    }

    public PathChain scorePreloads;
    public PathChain prepareSet2;
    public PathChain grabSet2;
    public PathChain prepareGate;
    public PathChain hitGate;
    public PathChain scoreSet2;
    public PathChain prepareSet3;
    public PathChain grabSet3;
    public PathChain scoreSet3;
    public PathChain prepareSet4;
    public PathChain grabSet4;
    public PathChain scoreSet4;

    public void buildPaths() {
        PedroComponent.follower().setStartingPose(new Pose(126, 118, Math.toRadians(36)));

        scorePreloads = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126, 118.300), new Pose(97,97))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();

        prepareSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97,97), new Pose(96.500, 83.800))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        grabSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 83.800), new Pose(124, 83.800))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        prepareGate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(124, 83.800), new Pose(123.000, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        hitGate = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123.000, 70.000), new Pose(125, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        scoreSet2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125, 70.000), new Pose(97,97))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        prepareSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97,97), new Pose(96.500, 58.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        grabSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 58), new Pose(131, 59.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scoreSet3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(131, 59.5),
                                new Pose(91.262, 56.240),
                                new Pose(95.176, 82.815),
                                new Pose(97,97)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        prepareSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97,97), new Pose(96.000, 38))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        grabSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.000, 38), new Pose(130, 37))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scoreSet4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130, 37), new Pose(90.000, 110.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(29))
                .build();

    }

    private Command scorePreloads() {
        return new SequentialGroup(
                //--------Preloads--------\\
                //Shoot Preloads
                new ParallelGroup(
                        new FollowPath(scorePreloads,true)
                        //SHOOTER FIRST SET SPEED
                        //AutonConceptSequences.INSTANCE.intakeSet(1060)
                )
                //AutonConceptSequences.INSTANCE.scoreSet(0.5,2.4)
                //new Delay(10)
        );
    }



    @Override
    public void onInit() {

        buildPaths();

    }

    @Override
    public void onStartButtonPressed() {

        scorePreloads().schedule();

    }




//            //--------Set 2--------\\
//                    //Grab Set 2
//            new ParallelGroup(
//                    new SequentialGroup(
//                            new FollowPath(prepareSet2),
//                            new FollowPath(grabSet2),
//                            new FollowPath(prepareGate),
//                            new FollowPath(hitGate)
//                    ),
//                    //SHOOTER SECOND SET SPEED
//                    AutonConceptSequences.INSTANCE.intakeSet(1080)
//            ),
//
//            //Shoot Set 2
//            new SequentialGroup(
//                    new FollowPath(scoreSet2, true),
//                    AutonConceptSequences.INSTANCE.scoreSet(0,2.6)
//            ),
//
//            //--------Set 3--------\\
//                    //Grab Set 3
//            new ParallelGroup(
//                    new SequentialGroup(
//                            new FollowPath(prepareSet3),
//                            new FollowPath(grabSet3)
//                    ),
//                    //SHOOTER THIRD SET SPEED
//                    AutonConceptSequences.INSTANCE.intakeSet(1080)
//            ),
//
//            //Shoot Set 3
//            new SequentialGroup(
//                    new FollowPath(scoreSet3, true),
//                    AutonConceptSequences.INSTANCE.scoreSet(0,2.6)
//            ),
//
//            //--------Set 4--------\\
//            //Grab Set 4
//            new ParallelGroup(
//                    new SequentialGroup(
//                            new FollowPath(prepareSet4),
//                            new FollowPath(grabSet4),
//                            VariableHood.INSTANCE.setHoodPos(0.52)
//                    ),
//                    //SHOOTER THIRD SET SPEED
//                    AutonConceptSequences.INSTANCE.intakeSet(990)
//            ),
//
//            //Shoot Set 4
//            new SequentialGroup(
//                    new FollowPath(scoreSet4, true),
//                    AutonConceptSequences.INSTANCE.scoreSet(0,2.6)
//            )


}