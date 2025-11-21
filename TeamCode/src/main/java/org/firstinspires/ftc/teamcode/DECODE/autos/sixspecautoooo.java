package org.firstinspires.ftc.teamcode.DECODE.autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;


import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "sixspec")
public class sixspecautoooo extends NextFTCOpMode {



    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Pose scorePose = new Pose(56, 8, Math.toRadians(90));
    private final Pose pickup1Pose = new Pose(36, 36, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup2Pose = new Pose(36, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose finishPose = new Pose(37.0, 50.0, Math.toRadians(180.0));

    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private Path preload;

    public void buildPaths() {

        preload = new Path(new BezierLine(scorePose, pickup1Pose));
        preload.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1Pose.getHeading())
                .build();




    }

    public Command secondRoutine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(grabPickup1)
                ),
                new ParallelGroup(

                )

        );
    }

    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(scorePose);
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
    }
}