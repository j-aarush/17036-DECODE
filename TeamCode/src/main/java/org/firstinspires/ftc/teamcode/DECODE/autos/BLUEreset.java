package org.firstinspires.ftc.teamcode.DECODE.autos;

import static org.firstinspires.ftc.teamcode.DECODE.botconstants.autoendpose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;


import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "BLUE RESET/park", preselectTeleOp = "BLUE TELEOP 2", group = "blueautos")
public class BLUEreset extends OpMode {

    public static Servo leftspindex, rightspindex;

    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    public static DcMotorEx intake, flywheel;
    public static float targetV = 1545;

    double kP = 0.12, kV = 0.00042;
    double error =0 ;

    float greenv, bluev, redv;
    double flickup = 0, flickdown = 0.53;
    double distancev;
    boolean move = false, intakeonoffb = false;
    boolean intakeswitch = false;
    int counter = 0;
    int shootercounter = 0;
    double rotationpos;


    int intaekstage = -1, shooterstage = -1, previntakestage = -1;

    double y;
    double x;
    double rx;

    // Denominator is the largest motor power (absolute value) or 1
    // This ensures all the powers maintain the same ratio,
    // but only if at least one is out of the range [-1, 1]
    double denominator;
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    DcMotorEx FL, FR, BL, BR, leftinake, rightinake;
    Servo flickys;

    GoBildaPinpointDriver pinpoint;

    private int pathState;


    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Pose startPose = new Pose(63.5, 8, Math.toRadians(90));

    private final Pose scorePose = new Pose(60, 14, Math.toRadians(109.25)); //figure outt
    private final Pose rescorePose = new Pose(59.75, 13.75, Math.toRadians(112.5)); //figure outt
    private final Pose prescorePose = new Pose(50.5, 20, Math.toRadians(180)); //figure outt
    private final Pose pickup1Pose = new Pose(20.22, 36.83, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose control = new Pose( 60.15, 37.81, Math.toRadians(180)); // Scoring Pose 2 of our robot. goes forward to intake
    private final Pose secondcontrol = new Pose(80, 59, Math.toRadians(180)); // Scoring Pose 2 of our robot. goes forward to intake

    private final Pose pickup2Pose = new Pose(23, 58, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(56.5, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose finishPose = new Pose(50.5, 25.0, Math.toRadians(108.0));

    private PathChain grabPickup1, return21, intake1, return1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, startshoot, return11, actuallyscorePickup2, park, resett;
    private Path grab1;
    public static class Paths {

        public PathChain initt;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            initt = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.273, 129.226), new Pose(24.125, 127.169))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(145))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.125, 127.169), new Pose(54.000, 81.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(145),
                            Math.toRadians(134.67)
                    )
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(54.000, 81.000),
                                    new Pose(64.332, 44.696),
                                    new Pose(27.117, 52.177),
                                    new Pose(25.621, 69.008),
                                    new Pose(9.000, 60.592)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(134.6), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.000, 60.592),
                                    new Pose(43.574, 49.745),
                                    new Pose(54.000, 81.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(134.67)
                    )
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(54.000, 81.000),
                                    new Pose(43.200, 75.740),
                                    new Pose(17.000, 84.904)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(134.6), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17.000, 84.904), new Pose(54.000, 81.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134.6))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(54.000, 81.000),
                                    new Pose(65.642, 26.743),
                                    new Pose(10.099, 35.719)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(134.6), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.099, 35.719), new Pose(54.000, 81.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(134.67)
                    )
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.000, 81.000), new Pose(38.338, 80.977))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(134.67),
                            Math.toRadians(170)
                    )
                    .build();
        }
    }


    public void buildPaths() {
        startshoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

//        grab1 = new Path(new BezierLine(scorePose, pickup1Pose));
//        grab1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, control, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        return1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, control, prescorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), prescorePose.getHeading())
                .build();

        return11 = follower.pathBuilder()
                .addPath(new BezierLine(prescorePose, scorePose))
                .setLinearHeadingInterpolation(prescorePose.getHeading(), scorePose.getHeading())
                .build();
        return21 = follower.pathBuilder()
                .addPath(new BezierLine(prescorePose, rescorePose))
                .setLinearHeadingInterpolation(prescorePose.getHeading(), rescorePose.getHeading())
                .build();


        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, secondcontrol, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, prescorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), prescorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finishPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), finishPose.getHeading())
                .build();
        resett = follower.pathBuilder()
                .addPath(new BezierLine(startPose, finishPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), finishPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                follower.followPath(resett,true );
                setPathState(-2);
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {

        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flickys = hardwareMap.get(Servo.class, "flicky");
        flickys.setDirection(Servo.Direction.FORWARD);

        leftspindex = hardwareMap.get(Servo.class, "leftspindex");
        rightspindex = hardwareMap.get(Servo.class, "rightspindex");
        intake = hardwareMap.get(DcMotorEx.class, "Lintake");


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setHeading(0, AngleUnit.DEGREES);

        // Configure the sensor
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        flickys.setPosition(flickup);
        flickys.setPosition(flickdown);
        settherotation(0.372); //first pos figure out later



    }

    @Override
    public void start() {
//        error = targetV - flywheel.getVelocity();
//        flywheel.setPower(kP * error + kV * targetV);
        opmodeTimer.resetTimer();
//        setPathState(-1);
    }

    @Override
    public void loop() {
//        error = targetV - flywheel.getVelocity();
//        flywheel.setPower(kP * error + kV * targetV);
        follower.update();
//        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("shooter vel", flywheel.getVelocity());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

    }

    @Override
    public void stop() {
        autoendpose = follower.getPose();
    }


}