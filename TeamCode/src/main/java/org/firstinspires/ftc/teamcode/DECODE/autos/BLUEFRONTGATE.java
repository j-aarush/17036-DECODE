package org.firstinspires.ftc.teamcode.DECODE.autos;

import static org.firstinspires.ftc.teamcode.DECODE.botconstants.autoendpose;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.autoendx;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.flickdown;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.flickup;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spina;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinb;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinc;
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


@Autonomous(name = "12 BLUE GATE FRONT", preselectTeleOp = "BLUE TELEOP 2", group = "blueautos")
public class BLUEFRONTGATE extends OpMode {

    public static Servo leftspindex, rightspindex;

    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    double diagonaldist;

    double headinglockangle;
    double turnerror;

    public static DcMotorEx intake, flywheel, sencoder;

    PIDFController controller = new PIDFController(new PIDFCoefficients(0.1,0,0.006,0.000004));

    public double         targetV = 1305;

    double kP = 0.111, kV = 0.0004349;
    double error;
    double botHeading;

    double posx ;
    double posy;
    boolean lefttoggle = false;
    double distx;
    double disty ;
    double trigangle;


    double spind = 1;


    float greenv, bluev, redv;
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
    private final Pose startPose = (new Pose(15.67, 113.5, Math.toRadians(180)));
    private final Pose realstartpose = (new Pose(24.025, 126.169, Math.toRadians(145)));
    private final Pose scorepose = (new Pose(49, 80, Math.toRadians(127.5)));
    private final Pose pickup1 = (new Pose(18, 68.75, Math.toRadians(180)));
    private final Pose pickup2 = (new Pose(22.1, 88, Math.toRadians(180)));
    private final Pose pickup3 = (new Pose(17, 60, Math.toRadians(138)));
    private final Pose parkpos = (new Pose(43, 77, Math.toRadians(140)));



    private PathChain score3rd, score3, initpath, score1, grabPickup1, score2, intake1, return1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, startshoot, return11, actuallyscorePickup2, pickup3rd, park;
    private Path grab1;

    public void buildPaths() {
        initpath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, realstartpose))
                .setLinearHeadingInterpolation(startPose.getHeading(), realstartpose.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorepose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorepose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorepose,
                        new Pose(63.77, 50.86),
                        new Pose(-50, 51),
                        new Pose(49.5, 64.9),
                        pickup1))
                .setConstantHeadingInterpolation((Math.toRadians(180)))
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1, new Pose(52, 63), scorepose))
                .setLinearHeadingInterpolation(pickup1.getHeading(), scorepose.getHeading())
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorepose, pickup2))
                .setConstantHeadingInterpolation((Math.toRadians(180)))
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2, scorepose))
                .setLinearHeadingInterpolation(pickup2.getHeading(), scorepose.getHeading())
                .build();
        pickup3rd = follower.pathBuilder()
                .addPath(new BezierCurve(scorepose, new Pose(24.5, 47), pickup3))
                .setLinearHeadingInterpolation((scorepose.getHeading()), (Math.toRadians(150)))
                .build();
        score3rd = follower.pathBuilder()
                .addPath(new BezierLine(pickup3, scorepose))
                .setLinearHeadingInterpolation((Math.toRadians(170)), scorepose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(scorepose, parkpos))
                .setLinearHeadingInterpolation(scorepose.getHeading(), parkpos.getHeading())
                .build();


    }


    void autonomousPathUpdate() {
        switch (pathState) {
            case -10:
                follower.followPath(initpath, true);
                setPathState(-9);
                break;
            case -9:
                if (!follower.isBusy()) {
                    follower.followPath(score1, true);
                    setPathState(-2); }
                break;
            case -2:
                if (pathTimer.getElapsedTimeSeconds()>2.5) {
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
                if (pathTimer.getElapsedTimeSeconds()>0.53) {
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
                if (pathTimer.getElapsedTimeSeconds()>0.67) {
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
                if(pathTimer.getElapsedTimeSeconds()>0.125) {
                    settherotation(spina);
                    follower.followPath(grabPickup1, 0.9, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(score2, true);
                    setPathState(-3);
                }
                break;
            case -3:
                if (!follower.isBusy()) {
                    settherotation(spina);
                    intake.setPower(-0.35);
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
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
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
                if (pathTimer.getElapsedTimeSeconds()>0.67) {
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
                    follower.followPath(pickup3rd, 0.99, true);
                    setPathState(1067);
                }
                break;
            case 1067:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(17);
                }

            case 17:
                if(pathTimer.getElapsedTimeSeconds() > 2)
                {
                    follower.followPath(score3rd,true);
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
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
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
                if (pathTimer.getElapsedTimeSeconds()>0.67) {
                    flickys.setPosition(flickup); //hopefully up]
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    flickys.setPosition(flickdown); //hopefully up]
                    setPathState(27);
                }
                break;




            case 27:
                if(pathTimer.getElapsedTimeSeconds()>0.15) {

                    settherotation(spina);
                    intake.setPower(1);
                    follower.followPath(grabPickup2, 0.75, true);
                    setPathState(28);
                }
                break;

            case 28:
                if(!follower.isBusy())
                {
                    follower.followPath(score3,true);
                    setPathState(29);
                }
                break;

            case 29:
                if (!follower.isBusy()) {
                    settherotation(spina);
                    intake.setPower(-0.35);
//                    follower.followPath(return21);
                    settherotation(spina);
                    setPathState(30);
                }
            case 30:
                if (pathTimer.getElapsedTimeSeconds() > 1.35 && !follower.isBusy()) {
                    flickys.setPosition(flickup); //hopefully up
                }
                if (pathTimer.getElapsedTimeSeconds()>1.55 && !follower.isBusy()) {
                    setPathState(31);
                    flickys.setPosition(flickdown); //hopefully up
                }

                break;
            case 31:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(32);
                    settherotation(spinb);
                }
                break;
            case 32:
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
                    setPathState(33);
                    flickys.setPosition(flickup); //hopefully up
                }
                break;
            case 33:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(34);
                    flickys.setPosition(flickdown); //hopefully up
                }
                break;
            case 34:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(35);
                    settherotation(spinc);
                }
                break;
            case 35:
                if (pathTimer.getElapsedTimeSeconds()>0.67) {
                    flickys.setPosition(flickup); //hopefully up]
                    setPathState(36);
                }
                break;
            case 36:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    flickys.setPosition(flickdown); //hopefully up]
                    setPathState(26);
                }
                break;
            case 26:
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(67);
                }
                break;
            case 67:
                if (!follower.isBusy()) {
                    autoendpose = follower.getPose();
                }

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
        sencoder = hardwareMap.get(DcMotorEx.class, "sencoder");
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
    public void init_loop() {
    }

    @Override
    public void start() {
        error = targetV - sencoder.getVelocity();
        flywheel.setPower(kP * error + kV * targetV);
        opmodeTimer.resetTimer();
        setPathState(-9);
    }

    @Override
    public void loop() {
        error = targetV - sencoder.getVelocity();
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

    @Override
    public void stop() {
//        follower.getPose();
        autoendpose = follower.getPose();
    }


}