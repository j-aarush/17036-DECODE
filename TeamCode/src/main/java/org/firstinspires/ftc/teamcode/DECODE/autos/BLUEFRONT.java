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


@Autonomous(name = "12 FRONT BLUE", preselectTeleOp = "NEWnewnewnew TELEOP")
public class BLUEFRONT extends OpMode {

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

    public double         targetV = 1325;

    double kP = 0.11, kV = 0.000435;
    double error;
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
    double flickup = 0, flickdown = 0.5;
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
    private final Pose startPose = (new Pose(39.273, 129.226, Math.toRadians(90)));
    private final Pose realstartpose = (new Pose(24.125, 127.169, Math.toRadians(145)));
    private final Pose scorepose = (new Pose(54, 81, Math.toRadians(134.67)));
    private final Pose pickup1 = (new Pose(9, 60.592, Math.toRadians(180)));
    private final Pose pickup2 = (new Pose(17, 84.904, Math.toRadians(180)));
    private final Pose pickup3 = (new Pose(10.099, 35.719, Math.toRadians(180)));
    private final Pose parkpos = (new Pose(38.338, 80.977, Math.toRadians(167)));



    private PathChain score3rd, score3, initpath, score1, grabPickup1, score2, intake1, return1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, startshoot, return11, actuallyscorePickup2, pickup3rd, park;
    private Path grab1;

    public void buildPaths() {
        initpath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, realstartpose))
                .setLinearHeadingInterpolation(startPose.getHeading(), realstartpose.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(realstartpose, scorepose))
                .setLinearHeadingInterpolation(realstartpose.getHeading(), scorepose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorepose,                                    new Pose(64.332, 44.696),
                        new Pose(27.117, 52.177),
                        new Pose(25.621, 69.008),
                        pickup1))
                .setLinearHeadingInterpolation(scorepose.getHeading(), pickup1.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1, new Pose(43.574, 49.745), scorepose))
                .setLinearHeadingInterpolation(pickup1.getHeading(), scorepose.getHeading())
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorepose, new Pose(43.2, 75.74), pickup2))
                .setLinearHeadingInterpolation(scorepose.getHeading(), pickup2.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2, scorepose))
                .setLinearHeadingInterpolation(pickup2.getHeading(), scorepose.getHeading())
                .build();
        pickup3rd = follower.pathBuilder()
                .addPath(new BezierCurve(scorepose, new Pose(65.642, 26.743), pickup3))
                .setLinearHeadingInterpolation(scorepose.getHeading(), pickup3.getHeading())
                .build();
        score3rd = follower.pathBuilder()
                .addPath(new BezierLine(pickup3, scorepose))
                .setLinearHeadingInterpolation(pickup3.getHeading(), scorepose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(scorepose, parkpos))
                .setLinearHeadingInterpolation(scorepose.getHeading(), parkpos.getHeading())
                .build();


    }


     void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(initpath);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(score1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(score2);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(score3);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(pickup3rd);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(score3rd);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(park);
                    setPathState(10);
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
        setPathState(1);
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