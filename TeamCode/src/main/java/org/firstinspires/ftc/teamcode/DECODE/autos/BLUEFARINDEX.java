package org.firstinspires.ftc.teamcode.DECODE.autos;

import static org.firstinspires.ftc.teamcode.DECODE.botconstants.autoendpose;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.flickdown;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.flickup;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.kP;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.kV;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.leftdown;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.leftup;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.rightdown;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.rightup;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spina;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinb;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinc;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spind;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spino;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.graphics.Color;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;


@Autonomous(name = "INDEXED 9 FAR BLUE", preselectTeleOp = "BLUE TELEOP 2", group = "blueautos")
public class BLUEFARINDEX extends OpMode {

    public static Servo leftspindex, rightspindex;

    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    public static DcMotorEx intake, flywheel, sencoder;
    public static float targetV = 1530;

    double error;

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
    private final Pose startPose = new Pose(63.5, 8, Math.toRadians(90));

    private final Pose scorePose = new Pose(60, 14, Math.toRadians(111.2)); //figure outt
    private final Pose rescorePose = new Pose(59.75, 13.75, Math.toRadians(111.2)); //figure outt 111.75
    private final Pose prescorePose = new Pose(50.5, 20, Math.toRadians(180)); //figure outt
    private final Pose pickup1Pose = new Pose(18.67, 36.83, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose control = new Pose( 55.15, 42.81, Math.toRadians(180)); // Scoring Pose 2 of our robot. goes forward to intake
    private final Pose secondcontrol = new Pose(80, 59, Math.toRadians(180)); // Scoring Pose 2 of our robot. goes forward to intake

    private final Pose pickup2Pose = new Pose(23, 58, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(56.5, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose finishPose = new Pose(50.5, 25.0, Math.toRadians(108.0));

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
                .setTValueConstraint(0.5)
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
                colorintake = intakecs.getNormalizedColors();
                colorleft = leftcs.getNormalizedColors();

                Color.colorToHSV(colorintake.toColor(), hsvValuesintake);
                Color.colorToHSV(colorleft.toColor(), hsvValuesleft);


                if (hsvValuesintake[0] > 130 && hsvValuesintake[0] < 176 && hsvValuesintake[2] > 0.019 && hsvValuesintake[2] < 0.041) {
                    rightisgreen = true;
                    noballright = false;
                    rightispurple = false;
                } else if (hsvValuesintake[0] > 179 && hsvValuesintake[0] < 230 && hsvValuesintake[2] > 0.015 && hsvValuesintake[2] < 0.033) {
                    rightisgreen = false;
                    noballright = false;
                    rightispurple = true;
                } else {
                    rightispurple = false;
                    rightisgreen = false;
                    noballright = true;
                }

                if (hsvValuesleft[0] > 130 && hsvValuesleft[0] < 176 && hsvValuesleft[2] > 0.019 && hsvValuesleft[2] < 0.041) {
                    leftisgreen = true;
                    noballleft = false;
                    leftispurple = false;
                } else if (hsvValuesleft[0] > 179 && hsvValuesleft[0] < 230 && hsvValuesleft[2] > 0.015 && hsvValuesleft[2] < 0.033) {
                    leftisgreen = false;
                    noballleft = false;
                    leftispurple = true;
                } else {
                    leftisgreen = false;
                    noballleft = true;
                    leftispurple = false;
                }

                if (hsvValuesleft[0] > 130 && hsvValuesleft[0] < 176 && hsvValuesleft[2] > 0.019 && hsvValuesleft[2] < 0.041) {
                    leftisgreen = true;
                    noballleft = false;
                } else leftisgreen = false;


                if (hsvValuesleft[0] > 179 && hsvValuesleft[0] < 230 && hsvValuesleft[2] > 0.015 && hsvValuesleft[2] < 0.031) {
                    leftispurple = true;
                    noballleft = false;
                } else leftispurple = false;

                if (leftispurple && rightispurple) {
                    PP = true;
                    PG = false;
                    GP = false;
                }
                if (leftispurple && rightisgreen) {
                    PP = false;
                    PG = true;
                    GP = false;
                }
                if (leftisgreen && rightispurple) {
                    PP = false;
                    PG = false;
                    GP = true;
                }
                if (noballleft || noballright) {
                    PP = true;
                    PG = false;
                    GP = true;
                }

                follower.followPath(startshoot,true );

                if (llresultt == 21 && PP) { //GPP
                    setPathState(-2);
                }
                else if (llresultt == 21 && PG) { //GPP
                    setPathState(-99);
                }
                else if (llresultt == 21 && GP) { //GPP
                    setPathState(-150);
                }
                else if (llresultt == 22 && PP) { //PGP
                    setPathState(-99);
                }
                else if (llresultt == 22 && PG) { //PGP
                    setPathState(-150);
                }
                else if (llresultt == 22 && GP) { //PGP
                    setPathState(-2);
                }
                else if (llresultt == 23 && PP) { //PPG
                    setPathState(-150);
                }
                else if (llresultt == 23 && PG) { //PPG
                    setPathState(-2);
                }
                else if (llresultt == 23 && GP) { //PPG
                    setPathState(-99);
                }
                else {
                    setPathState(-2);
                }

                setPathState(-2);
                break;


            case -150:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    settherotation(spinb);
                    setPathState(-151);
                }
                break;
            case -151:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    setPathState(-152);
                    spinflickup();
                }
                break;
            case -152:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(-153);
                    spinflickdown();
                }

                break;
            case -153:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(-154);
                    settherotation(spinc);
                }
                break;
            case -154:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(-155);
                    spinflickup();
                }
                break;
            case -155:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(-156);
                    spinflickdown();
                }
                break;
            case -156:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(-157);
                    settherotation(spind);
                }
                break;
            case -157:
                if (pathTimer.getElapsedTimeSeconds()>0.8) {
                    setPathState(-158);
                    spinflickup();
                }
                break;
            case -158:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(7);
                    spinflickdown();
                    intake.setPower(1);
                }
                break;




            case -99:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    settherotation(spino);
                    setPathState(-100);
                }
                break;
            case -100:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    setPathState(-98);
                    spinflickup();
                }
                break;
            case -98:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(-97);
                    spinflickdown();
                }

                break;
            case -97:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(-96);
                    settherotation(spina);
                }
                break;
            case -96:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(-95);
                    spinflickup();
                }
                break;
            case -95:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(-94);
                    spinflickdown();
                }
                break;
            case -94:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(-93);
                    settherotation(spinb);
                }
                break;
            case -93:
                if (pathTimer.getElapsedTimeSeconds()>0.8) {
                    setPathState(-92);
                    spinflickup();
                }
                break;
            case -92:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(7);
                    spinflickdown();
                    intake.setPower(1);
                }
                break;



            case -2:
                if (pathTimer.getElapsedTimeSeconds()>2.5) {
                    setPathState(0);
                    spinflickup();
                }
                break;
            case 0:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(1);
                    spinflickdown();
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
                    spinflickup();
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(4);
                    spinflickdown();
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
                    spinflickup();
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(7);
                    spinflickdown();
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

                    setPathState(90);
                }

            case 90:
                colorintake = intakecs.getNormalizedColors();
                colorleft = leftcs.getNormalizedColors();

                Color.colorToHSV(colorintake.toColor(), hsvValuesintake);
                Color.colorToHSV(colorleft.toColor(), hsvValuesleft);


                if (hsvValuesintake[0] > 130 && hsvValuesintake[0] < 176 && hsvValuesintake[2] > 0.019 && hsvValuesintake[2] < 0.041) {
                    rightisgreen = true;
                    noballright = false;
                    rightispurple = false;
                } else if (hsvValuesintake[0] > 179 && hsvValuesintake[0] < 230 && hsvValuesintake[2] > 0.015 && hsvValuesintake[2] < 0.033) {
                    rightisgreen = false;
                    noballright = false;
                    rightispurple = true;
                } else {
                    rightispurple = false;
                    rightisgreen = false;
                    noballright = true;
                }

                if (hsvValuesleft[0] > 130 && hsvValuesleft[0] < 176 && hsvValuesleft[2] > 0.019 && hsvValuesleft[2] < 0.041) {
                    leftisgreen = true;
                    noballleft = false;
                    leftispurple = false;
                } else if (hsvValuesleft[0] > 179 && hsvValuesleft[0] < 230 && hsvValuesleft[2] > 0.015 && hsvValuesleft[2] < 0.033) {
                    leftisgreen = false;
                    noballleft = false;
                    leftispurple = true;
                } else {
                    leftisgreen = false;
                    noballleft = true;
                    leftispurple = false;
                }

                if (hsvValuesleft[0] > 130 && hsvValuesleft[0] < 176 && hsvValuesleft[2] > 0.019 && hsvValuesleft[2] < 0.041) {
                    leftisgreen = true;
                    noballleft = false;
                } else leftisgreen = false;


                if (hsvValuesleft[0] > 179 && hsvValuesleft[0] < 230 && hsvValuesleft[2] > 0.015 && hsvValuesleft[2] < 0.031) {
                    leftispurple = true;
                    noballleft = false;
                } else leftispurple = false;

                if (leftispurple && rightispurple) {
                    PP = true;
                    PG = false;
                    GP = false;
                }
                if (leftispurple && rightisgreen) {
                    PP = false;
                    PG = true;
                    GP = false;
                }
                if (leftisgreen && rightispurple) {
                    PP = false;
                    PG = false;
                    GP = true;
                }
                if (noballleft || noballright) {
                    PP = true;
                    PG = false;
                    GP = true;
                }

                follower.followPath(startshoot,true );

                if (llresultt == 21 && PP) { //GPP
                    setPathState(9);
                }
                else if (llresultt == 21 && PG) { //GPP
                    setPathState(99);
                }
                else if (llresultt == 21 && GP) { //GPP
                    setPathState(150);
                }
                else if (llresultt == 22 && PP) { //PGP
                    setPathState(99);
                }
                else if (llresultt == 22 && PG) { //PGP
                    setPathState(150);
                }
                else if (llresultt == 22 && GP) { //PGP
                    setPathState(9);
                }
                else if (llresultt == 23 && PP) { //PPG
                    setPathState(150);
                }
                else if (llresultt == 23 && PG) { //PPG
                    setPathState(-2);
                }
                else if (llresultt == 23 && GP) { //PPG
                    setPathState(99);
                }
                else {
                    setPathState(9);
                }

                setPathState(9);
                break;


            case 150:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    settherotation(spinb);
                    setPathState(151);
                }
                break;
            case 151:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    setPathState(152);
                    spinflickup();
                }
                break;
            case 152:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(153);
                    spinflickdown();
                }

                break;
            case 153:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(154);
                    settherotation(spinc);
                }
                break;
            case 154:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(155);
                    spinflickup();
                }
                break;
            case 155:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(156);
                    spinflickdown();
                }
                break;
            case 156:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(157);
                    settherotation(spind);
                }
                break;
            case 157:
                if (pathTimer.getElapsedTimeSeconds()>0.8) {
                    setPathState(158);
                    spinflickup();
                }
                break;
            case 158:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(16);
                    spinflickdown();
                    intake.setPower(1);
                }
                break;




            case 99:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    settherotation(spino);
                    setPathState(100);
                }
                break;
            case 100:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    setPathState(98);
                    spinflickup();
                }
                break;
            case 98:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(97);
                    spinflickdown();
                }

                break;
            case 97:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(96);
                    settherotation(spina);
                }
                break;
            case 96:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(95);
                    spinflickup();
                }
                break;
            case 95:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(94);
                    spinflickdown();
                }
                break;
            case 94:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(93);
                    settherotation(spinb);
                }
                break;
            case 93:
                if (pathTimer.getElapsedTimeSeconds()>0.8) {
                    setPathState(92);
                    spinflickup();
                }
                break;
            case 92:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(16);
                    spinflickdown();
                    intake.setPower(1);
                }
                break;







            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1.1 && !follower.isBusy()) {
                    spinflickup();
                }
                if (pathTimer.getElapsedTimeSeconds()>1.30 && !follower.isBusy()) {
                    setPathState(10);
                    spinflickdown();
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
                    spinflickup();
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(13);
                    spinflickdown();
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
                    spinflickup();
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    spinflickdown();
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
                    setPathState(180);
                }
                break;


            case 180:
                colorintake = intakecs.getNormalizedColors();
                colorleft = leftcs.getNormalizedColors();

                Color.colorToHSV(colorintake.toColor(), hsvValuesintake);
                Color.colorToHSV(colorleft.toColor(), hsvValuesleft);


                if (hsvValuesintake[0] > 130 && hsvValuesintake[0] < 176 && hsvValuesintake[2] > 0.019 && hsvValuesintake[2] < 0.041) {
                    rightisgreen = true;
                    noballright = false;
                    rightispurple = false;
                } else if (hsvValuesintake[0] > 179 && hsvValuesintake[0] < 230 && hsvValuesintake[2] > 0.015 && hsvValuesintake[2] < 0.033) {
                    rightisgreen = false;
                    noballright = false;
                    rightispurple = true;
                } else {
                    rightispurple = false;
                    rightisgreen = false;
                    noballright = true;
                }

                if (hsvValuesleft[0] > 130 && hsvValuesleft[0] < 176 && hsvValuesleft[2] > 0.019 && hsvValuesleft[2] < 0.041) {
                    leftisgreen = true;
                    noballleft = false;
                    leftispurple = false;
                } else if (hsvValuesleft[0] > 179 && hsvValuesleft[0] < 230 && hsvValuesleft[2] > 0.015 && hsvValuesleft[2] < 0.033) {
                    leftisgreen = false;
                    noballleft = false;
                    leftispurple = true;
                } else {
                    leftisgreen = false;
                    noballleft = true;
                    leftispurple = false;
                }

                if (hsvValuesleft[0] > 130 && hsvValuesleft[0] < 176 && hsvValuesleft[2] > 0.019 && hsvValuesleft[2] < 0.041) {
                    leftisgreen = true;
                    noballleft = false;
                } else leftisgreen = false;


                if (hsvValuesleft[0] > 179 && hsvValuesleft[0] < 230 && hsvValuesleft[2] > 0.015 && hsvValuesleft[2] < 0.031) {
                    leftispurple = true;
                    noballleft = false;
                } else leftispurple = false;

                if (leftispurple && rightispurple) {
                    PP = true;
                    PG = false;
                    GP = false;
                }
                if (leftispurple && rightisgreen) {
                    PP = false;
                    PG = true;
                    GP = false;
                }
                if (leftisgreen && rightispurple) {
                    PP = false;
                    PG = false;
                    GP = true;
                }
                if (noballleft || noballright) {
                    PP = true;
                    PG = false;
                    GP = true;
                }

                follower.followPath(startshoot,true );

                if (llresultt == 21 && PP) { //GPP
                    setPathState(18);
                }
                else if (llresultt == 21 && PG) { //GPP
                    setPathState(-990);
                }
                else if (llresultt == 21 && GP) { //GPP
                    setPathState(-1500);
                }
                else if (llresultt == 22 && PP) { //PGP
                    setPathState(-990);
                }
                else if (llresultt == 22 && PG) { //PGP
                    setPathState(-1500);
                }
                else if (llresultt == 22 && GP) { //PGP
                    setPathState(18);
                }
                else if (llresultt == 23 && PP) { //PPG
                    setPathState(-1500);
                }
                else if (llresultt == 23 && PG) { //PPG
                    setPathState(18);
                }
                else if (llresultt == 23 && GP) { //PPG
                    setPathState(-990);
                }
                else {
                    setPathState(18);
                }

                setPathState(18);
                break;


            case -1500:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    settherotation(spinb);
                    setPathState(-1510);
                }
                break;
            case -1510:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    setPathState(-1520);
                    spinflickup();
                }
                break;
            case -1520:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(-1530);
                    spinflickdown();
                }

                break;
            case -1530:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(-1540);
                    settherotation(spinc);
                }
                break;
            case -1540:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(-1550);
                    spinflickup();
                }
                break;
            case -1550:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(-1560);
                    spinflickdown();
                }
                break;
            case -1560:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(-1570);
                    settherotation(spind);
                }
                break;
            case -1570:
                if (pathTimer.getElapsedTimeSeconds()>0.8) {
                    setPathState(-1580);
                    spinflickup();
                }
                break;
            case -1580:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(26);
                    spinflickdown();
                    intake.setPower(1);
                }
                break;




            case -990:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    settherotation(spino);
                    setPathState(-1000);
                }
                break;
            case -1000:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    setPathState(-980);
                    spinflickup();
                }
                break;
            case -980:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(-970);
                    spinflickdown();
                }

                break;
            case -970:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(-960);
                    settherotation(spina);
                }
                break;
            case -960:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(-950);
                    spinflickup();
                }
                break;
            case -950:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(-940);
                    spinflickdown();
                }
                break;
            case -940:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(-930);
                    settherotation(spinb);
                }
                break;
            case -930:
                if (pathTimer.getElapsedTimeSeconds()>0.8) {
                    setPathState(-920);
                    spinflickup();
                }
                break;
            case -920:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(26);
                    spinflickdown();
                    intake.setPower(1);
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
                    spinflickup();
                }
                if (pathTimer.getElapsedTimeSeconds()>1.55 && !follower.isBusy()) {
                    setPathState(20);
                    spinflickdown();
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
                    spinflickup();
                }
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(23);
                    spinflickdown();
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
                    spinflickup();
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    spinflickdown();
                    setPathState(26);
                }
                break;
            case 26:
                if (!follower.isBusy()) {
                    follower.followPath(park);
                    setPathState(67);
                }
                break;
            case 67:
                if (!follower.isBusy()) {
                    autoendpose = follower.getPose();
                }

        }
    }

///SORTING/NEW STUFF:

    boolean skiponee, reverseonee, dononee;
    float greenintake, blueintake, greenleft, blueleft, greenright, blueright;
    NormalizedColorSensor intakecs, leftcs;
    final float[] hsvValuesintake = new float[3];
    final float[] hsvValuesright = new float[3];
    final float[] hsvValuesleft = new float[3];
    boolean leftisgreen, leftispurple, rightisgreen, rightispurple, noballleft, noballright;
    boolean PP, PG, GP;

    Servo flickright, leftwall, rightwall;
    public void spinflickup() {
        flickys.setPosition(flickup);
        flickright.setPosition(flickup);
    }
    public void spinflickdown() {
        flickys.setPosition(flickdown);
        flickright.setPosition(flickdown);
    }
    NormalizedRGBA colorintake, colorleft;

    public void setleftdown() {
        leftwall.setPosition(leftdown);
        rightwall.setPosition(rightup);
    }
    public void setrightdown() {
        leftwall.setPosition(leftup);
        rightwall.setPosition(rightdown);
    }
    public void bothwalldown() {
        leftwall.setPosition(leftdown);
        rightwall.setPosition(rightdown);
    }
    public void bothwallup() {
        leftwall.setPosition(leftup);
        rightwall.setPosition(rightup);
    }


    public Limelight3A limelight;
    int llresultt;

    public int getPatternIdAuto() { // only for auto just returns the tag id for patterns
        this.limelight.pipelineSwitch(0);
        LLResult result = this.limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                return fiducial.getFiducialId();
            }
        }
        return 0;
    }
    int skip1 = -1, reverse1 = -1, thedefault = -1;




    ///END SORTING/NEW STUFF


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
        sencoder = hardwareMap.get(DcMotorEx.class, "sencoder");

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
        follower.setMaxPower(0.9);

        settherotation(spina); //first pos figure out later

        limelight = hardwareMap.get(Limelight3A.class, "lime");

        intakecs = hardwareMap.get(NormalizedColorSensor.class, "intakecolor");
        leftcs = hardwareMap.get(NormalizedColorSensor.class, "leftcs");

        flickright = hardwareMap.get(Servo.class, "flickyr");
        flickright.setDirection(Servo.Direction.REVERSE);


        rightwall = hardwareMap.get(Servo.class, "rightwall");
        leftwall = hardwareMap.get(Servo.class, "leftwall");

        llresultt = getPatternIdAuto();
    }

    @Override
    public void start() {
        error = targetV - sencoder.getVelocity();
        flywheel.setPower(kP * error + kV * targetV);
        opmodeTimer.resetTimer();
        setPathState(-1);
    }

    @Override
    public void loop() {
        error = targetV - sencoder.getVelocity();
        flywheel.setPower(kP * error + kV * targetV);
        follower.update();
        autonomousPathUpdate();
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