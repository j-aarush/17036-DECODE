package org.firstinspires.ftc.teamcode.DECODE.autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
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


@Autonomous(name = "9 AUTO RED", preselectTeleOp = "NEWnewnewnew TELEOP")
public class sixspecautooored extends OpMode {

    public static Servo leftspindex, rightspindex;

    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    double diagonaldist;

    double headinglockangle;
    double turnerror;

    public static DcMotorEx intake, flywheel;

    PIDFController controller = new PIDFController(new PIDFCoefficients(0.1,0,0.006,0.000004));

    public double         targetV = 1535;

    double kP = 0.11, kV = 0.000435;
    double error =0 ;
    double botHeading;

    double posx ;
    double posy;
    boolean lefttoggle = false;
    double distx;
    double disty ;
    double trigangle;

    double spina = 0.233;
    double spinb = 0.49;
    double spinc = 0.743;

    double spind = 1;


    float greenv, bluev, redv;
    double flickup = 0.045, flickdown = 0.5;
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
    private final Pose startPose = new Pose(63.5, 8, Math.toRadians(90)).mirror();

    private final Pose scorePose = new Pose(60, 14, Math.toRadians(110)).mirror(); //figure outt
    private final Pose rescorePose = new Pose(60.25, 14.25, Math.toRadians(111)).mirror();

    private final Pose prescorePose = new Pose(50.5, 20, Math.toRadians(150)).mirror(); //figure outt
    private final Pose pickup1Pose = new Pose(22, 38, Math.toRadians(180)).mirror(); // 19.7 x Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose control = new Pose( 60.15, 40, Math.toRadians(180)).mirror(); // Scoring Pose 2 of our robot. goes forward to intake
    private final Pose secondcontrol = new Pose(78, 65, Math.toRadians(180)).mirror(); // Scoring Pose 2 of our robot. goes forward to intake

    private final Pose pickup2Pose = new Pose(20, 59, Math.toRadians(180)).mirror(); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(56.5, 135, Math.toRadians(0)).mirror(); // Lowest (Third Set) of Artifacts from the Spike Mark.

    static final Pose finishPose = new Pose(50.5, 25.0, Math.toRadians(108.0)).mirror();

    private PathChain grabPickup1, return21, intake1, return1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, startshoot, return11, actuallyscorePickup2, park;
    private Path grab1;

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
                .addPath(new BezierCurve(pickup1Pose, control, rescorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), rescorePose.getHeading())
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
                .addPath(new BezierLine(pickup2Pose, rescorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), rescorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finishPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), finishPose.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                follower.followPath(startshoot,true );
                setPathState(-2);

                break;
            case -2:
                if (pathTimer.getElapsedTimeSeconds()>2.75) {
                    setPathState(0);
                    flickys.setPosition(flickup);
                }
                break;
            case 0:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(1);
                    flickys.setPosition(flickdown); //hopefully up
                }

                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(2);
                    settherotation(spinb);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(3);
                    flickys.setPosition(flickup); //hopefully up
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(4);
                    flickys.setPosition(flickdown); //hopefully up
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(5);
                    settherotation(spinc);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds()>0.8) {
                    setPathState(6);
                    flickys.setPosition(flickup); //hopefully up
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(7);
                    flickys.setPosition(flickdown); //hopefully up
                    intake.setPower(1);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds()>0.15) {

                    settherotation(spina);
                    follower.followPath(intake1, true);
                    setPathState(-8);
                }

                break;

            case -8:
                if(!follower.isBusy())
                {
                    follower.followPath(return1,true);
                    setPathState(-10);
                }
                break;

            case -10:
                if (!follower.isBusy()) {
                    settherotation(spina);
                    intake.setPower(-0.35);
//                    follower.followPath(return21);
                    settherotation(spina);

                    setPathState(9);
                }
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1.1 && !follower.isBusy()) {
                    flickys.setPosition(flickup); //hopefully up
                }
                if (pathTimer.getElapsedTimeSeconds()>1.30 && !follower.isBusy()) {
                    setPathState(10);
                    flickys.setPosition(flickdown); //hopefully up
                }

                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds()>0.1) {
                    setPathState(11);
                    settherotation(spinb);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(12);
                    flickys.setPosition(flickup); //hopefully up
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(13);
                    flickys.setPosition(flickdown); //hopefully up
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(14);
                    settherotation(spinc);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds()>0.80) {
                    flickys.setPosition(flickup); //hopefully up]
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    flickys.setPosition(flickdown); //hopefully up]
                    setPathState(16);
                }
                break;





            case 16:
                if(pathTimer.getElapsedTimeSeconds()>0.15) {

                    settherotation(spina);
                    intake.setPower(1);
                    follower.followPath(grabPickup2, true);
                    setPathState(17);
                }
                break;

            case 17:
                if(!follower.isBusy())
                {
                    follower.followPath(return1,true);
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    settherotation(spina);
                    intake.setPower(-0.35);
//                    follower.followPath(return21);
                    settherotation(spina);
                    setPathState(19);
                }
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 1.35 && !follower.isBusy()) {
                    flickys.setPosition(flickup); //hopefully up
                }
                if (pathTimer.getElapsedTimeSeconds()>1.55 && !follower.isBusy()) {
                    setPathState(20);
                    flickys.setPosition(flickdown); //hopefully up
                }

                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(21);
                    settherotation(spinb);
                }
                break;
            case 21:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(22);
                    flickys.setPosition(flickup); //hopefully up
                }
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(23);
                    flickys.setPosition(flickdown); //hopefully up
                }
                break;
            case 23:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(24);
                    settherotation(spinc);
                }
                break;
            case 24:
                if (pathTimer.getElapsedTimeSeconds()>0.80) {
                    flickys.setPosition(flickup); //hopefully up]
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    flickys.setPosition(flickdown); //hopefully up]
                    setPathState(26);
                }
                break;
            case 26:
                if (!follower.isBusy()) {
                    follower.followPath(park);
                    setPathState(67);
                }
                break;
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
        settherotation(spina); //first pos figure out later



    }

    @Override
    public void start() {
        error = targetV - flywheel.getVelocity();
        flywheel.setPower(kP * error + kV * targetV);
        opmodeTimer.resetTimer();
        setPathState(-1);
    }

    @Override
    public void loop() {
        error = targetV - flywheel.getVelocity();
//        botHeading = follower.getHeading();
//        posx = follower.getPose().mirror().getX();
//        posy = follower.getPose().mirror().getY();
//        distx = posx - 9;
//        disty = Math.abs(137 - posy);
//        diagonaldist = Math.sqrt(distx*distx + disty*disty);
//        trigangle = Math.toDegrees(Math.atan(disty/distx));
//        headinglockangle = trigangle;
//        turnerror = headinglockangle - Math.toDegrees(botHeading);
//        controller.updateError(turnerror);


        flywheel.setPower(kP * error + kV * targetV);
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

    }



}