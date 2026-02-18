package org.firstinspires.ftc.teamcode.DECODE.teleops;

import static org.firstinspires.ftc.teamcode.DECODE.botconstants.autoendpose;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.flickdown;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.flickup;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.leftdown;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.leftup;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.rightdown;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.rightup;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spina;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinb;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinc;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spind;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spino;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import java.util.List;


@TeleOp(name = "INDEX RED TELEOP 2")
@Configurable
public class INDEXPEDROTELEOPREDTWO extends NextFTCOpMode {

    boolean skiponee, reverseonee, dononee;
    private Follower follower;
    float greenintake, blueintake, greenleft, blueleft, greenright, blueright;
    NormalizedColorSensor intakecs, leftcs;
    final float[] hsvValuesintake = new float[3];
    final float[] hsvValuesright = new float[3];
    final float[] hsvValuesleft = new float[3];
    boolean leftisgreen, leftispurple, rightisgreen, rightispurple, noballleft, noballright;
    boolean PP, PG, GP;

    Boolean intakepressed = false;
    Boolean intaketoggle;

    private boolean automatedDrive;
    boolean holdshooting = false;
    private PathChain pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    double headinglockangle;


    double posx ;
    double posy;
    boolean lefttoggle = false;
    double distx;
    double disty ;
    double diagonaldist;
    double trigangle;

    public static Servo leftspindex, rightspindex;

    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    public static DcMotorEx intake, flywheel, sencoder;
    public static double targetV = 0;

    double kP = 0.1167, kV = 0.000434;
    double error;

    float greenv, bluev, redv;
    double distancev;
    boolean move = false, intakeonoffb = false;
    boolean intakeswitch = false;
    int counter = 0;
    int shootercounter = 0;
    double rotationpos;
    Servo flickys, flickright;

    public void spinflickup() {
        flickys.setPosition(flickup);
        flickright.setPosition(flickup);
    }
    public void spinflickdown() {
        flickys.setPosition(flickdown);
        flickright.setPosition(flickdown);
    }
    NormalizedRGBA colorintake, colorleft;

    double turnerror;
    public Servo leftpark, rightpark, leftwall, rightwall;

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


    public void parksettherotation(double rotationn) {
        leftpark.setPosition(rotationn);
        rightpark.setPosition(rotationn);
    }
    PIDFController controller = new PIDFController(new PIDFCoefficients(0.1,0,0.006,0.000004));


    int intaekstage = -1, shooterstage = -1, previntakestage = -1;


    boolean headingLock;

    DcMotorEx FL, FR, BL, BR, leftinake, rightinake;

    double botHeading;

    public static ElapsedTime intakeeee = new ElapsedTime(0);

    public static ElapsedTime getIntakeeee() {
        return intakeeee;
    }

    double intaketimercount;
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

//call method forever. when need to use, set to 5 once. call case 4 when identifying pattern/order to prepare. case 5 when to shoot.
    //THE TWO LETTER PHRASE REFERS TO THE TWO BALLS IDENTIFIED BY COLOR SENSOR. FIRST LETTER IS BALL ON LEFT, SECOND IS BALL ON RIGHT.

    public void sortone() { //TS WORKS FOR THE FOLLOWING COMBOS: PGP AND PG; PPG AND PP; GPP AND GP
        switch (skip1) {
            case 4:
                bothwallup();
                settherotation(spinb);
                intakeeee.reset();
                skip1 = 3;
                break;
            case 3:
                if (intakeeee.time() > 0.5) {
                    setrightdown();
                }
                break;
            case 5:
                if (intakeeee.time() > 0.0005) {
                    headingLock = true;
                    bothwalldown();
                    skip1 = 6;
                    intakeeee.reset();}
                break;
            case 6:
                settherotation(spinb);
                if (!follower.isBusy()) {
                    skip1 = 7;
                    intakeeee.reset();}
                break;
            case 7:
                if (holdshooting) {
                    parksettherotation(0.1);
                }
                spinflickup();
                if (intakeeee.time() > 0.056) {
                    skip1 = 8;
                    intakeeee.reset();}
                break;
            case 8:
                spinflickdown();
                setrightdown();
                if (intakeeee.time() > 0.056) {
                    skip1 = 9;
                    intakeeee.reset();}
                break;
            case 9:
                settherotation(spinc);
                if (intakeeee.time() > 0.56) {
                    skip1 = 10;
                    intakeeee.reset();}
                break;
            case 10:
                spinflickup();
                if (intakeeee.time() > 0.056) {
                    skip1 = 11;
                    intakeeee.reset();}
                break;
            case 11:
                spinflickdown();
                if (intakeeee.time() > 0.056) {
                    skip1 = 12;
                    intakeeee.reset();}
                break;
            case 12:
                settherotation(spind);
                if (intakeeee.time() > 0.57) {
                    skip1 = 13;
                    intakeeee.reset();}
                break;
            case 13:
                spinflickup();
                if (intakeeee.time() > 0.65) {
                    headingLock = false;
                    skip1 = 14;
                    intakeeee.reset();}
                break;
            case 14:
                parksettherotation(0);
                headingLock = false;
                spinflickdown();
                if (intakeeee.time() > 0.056) {
                    skip1 = -1;
                    intakeeee.reset();
                    settherotation(spina);}
                break;

        }
    }
    public void sorttwo() { //TS WORKS FOR THE FOLLOWING COMBOS: PGP AND PP; PPG AND GP; GPP AND PG;
        switch (reverse1) {
            case 4:
                bothwallup();
                settherotation(spino);
                intakeeee.reset();
                reverse1 = 3;
                break;
            case 3:
                if (intakeeee.time() > 0.5) {
                    setleftdown();
                }
                break;
            case 5:
                if (intakeeee.time() > 0.0005) {
                    headingLock = true;
                    bothwalldown();
                    reverse1 = 6;
                    intakeeee.reset();}
                break;
            case 6:
                settherotation(spino);
                if (!follower.isBusy()) {
                    reverse1 = 7;
                    intakeeee.reset();}
                break;
            case 7:
                if (holdshooting) {
                    parksettherotation(0.1);
                }
                spinflickup();
                if (intakeeee.time() > 0.056) {
                    reverse1 = 8;
                    intakeeee.reset();}
                break;
            case 8:
                spinflickdown();
                setrightdown();
                if (intakeeee.time() > 0.056) {
                    reverse1 = 9;
                    intakeeee.reset();}
                break;
            case 9:
                settherotation(spina);
                if (intakeeee.time() > 0.56) {
                    reverse1 = 10;
                    intakeeee.reset();}
                break;
            case 10:
                spinflickup();
                if (intakeeee.time() > 0.056) {
                    reverse1 = 11;
                    intakeeee.reset();}
                break;
            case 11:
                spinflickdown();
                if (intakeeee.time() > 0.056) {
                    reverse1 = 12;
                    intakeeee.reset();}
                break;
            case 12:
                settherotation(spinb);
                if (intakeeee.time() > 0.57) {
                    reverse1 = 13;
                    intakeeee.reset();}
                break;
            case 13:
                spinflickup();
                if (intakeeee.time() > 0.65) {
                    headingLock = false;
                    reverse1 = 14;
                    intakeeee.reset();}
                break;
            case 14:
                parksettherotation(0);
                headingLock = false;
                spinflickdown();
                if (intakeeee.time() > 0.056) {
                    reverse1 = -1;
                    intakeeee.reset();
                    settherotation(spina);}
                break;

        }
    }
    public void sortthree() { //TS WORKS FOR THE FOLLOWING COMBOS: PGP AND GP; PPG AND PG; GPP AND PP;
        switch (thedefault) {
            case 4:
                setrightdown();
                break;
            case 5:
                if (intakeeee.time() > 0.0005) {
                    headingLock = true;
                    setrightdown();
                    thedefault = 6;
                    intakeeee.reset();}
                break;
            case 6:
                settherotation(spina);
                if (!follower.isBusy()) {
                    thedefault = 7;
                    intakeeee.reset();}
                break;
            case 7:
                if (holdshooting) {
                    parksettherotation(0.1);
                }
                spinflickup();
                if (intakeeee.time() > 0.056) {
                    thedefault = 8;
                    intakeeee.reset();}
                break;
            case 8:
                spinflickdown();
                if (intakeeee.time() > 0.056) {
                    thedefault = 9;
                    intakeeee.reset();}
                break;
            case 9:
                settherotation(spinb);
                if (intakeeee.time() > 0.56) {
                    thedefault = 10;
                    intakeeee.reset();}
                break;
            case 10:
                spinflickup();
                if (intakeeee.time() > 0.056) {
                    thedefault = 11;
                    intakeeee.reset();}
                break;
            case 11:
                spinflickdown();
                if (intakeeee.time() > 0.056) {
                    thedefault = 12;
                    intakeeee.reset();}
                break;
            case 12:
                settherotation(spinc);
                if (intakeeee.time() > 0.57) {
                    thedefault = 13;
                    intakeeee.reset();}
                break;
            case 13:
                spinflickup();
                if (intakeeee.time() > 0.65) {
                    headingLock = false;
                    thedefault = 14;
                    intakeeee.reset();}
                break;
            case 14:
                parksettherotation(0);
                headingLock = false;
                spinflickdown();
                if (intakeeee.time() > 0.056) {
                    thedefault = -1;
                    intakeeee.reset();
                    settherotation(spina);}
                break;

        }
    }



    @Override
    public void onInit() {

        limelight = hardwareMap.get(Limelight3A.class, "lime");

        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        sencoder = hardwareMap.get(DcMotorEx.class, "sencoder");
        FR = hardwareMap.get(DcMotorEx.class, "BR");
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakecs = hardwareMap.get(NormalizedColorSensor.class, "intakecolor");
        leftcs = hardwareMap.get(NormalizedColorSensor.class, "leftcs");

        flickys = hardwareMap.get(Servo.class, "flicky");
        flickright = hardwareMap.get(Servo.class, "flickyr");
        flickys.setDirection(Servo.Direction.FORWARD);
        flickright.setDirection(Servo.Direction.REVERSE);

        leftspindex = hardwareMap.get(Servo.class, "leftspindex");
        rightspindex = hardwareMap.get(Servo.class, "rightspindex");

        intake = hardwareMap.get(DcMotorEx.class, "Lintake");

        leftpark = hardwareMap.get(Servo.class, "leftpark");
        rightpark = hardwareMap.get(Servo.class, "rightpark");
        leftpark.setDirection(Servo.Direction.FORWARD);
        rightpark.setDirection(Servo.Direction.REVERSE);
        rightwall = hardwareMap.get(Servo.class, "rightwall");
        leftwall = hardwareMap.get(Servo.class, "leftwall");




        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(50.5, 25.0, Math.toRadians(108.0)).mirror());
        follower.setStartingPose(autoendpose);
        follower.update();
        headingLock = false;

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, new Pose(42,35)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(270), 0.8))
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        intakeeee.reset();
        follower.startTeleopDrive();
        follower.update();
        limelight.start();
    }

    @Override
    public void onUpdate() {
        follower.update();

        botHeading = follower.getHeading();
        posx = follower.getPose().mirror().getX();
        posy = follower.getPose().mirror().getY();
        distx = posx - 9;
        disty = Math.abs(137 - posy);
        diagonaldist = Math.sqrt(distx*distx + disty*disty);
        trigangle = Math.toDegrees(Math.atan(disty/distx));
        headinglockangle = trigangle;


        error = targetV - sencoder.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);


        if (gamepad1.b) {
            targetV = 0;
        } else {
            targetV = 2447 + -51.2*diagonaldist + 0.753*diagonaldist*diagonaldist + -0.00437*diagonaldist*diagonaldist*diagonaldist + 0.0000091*diagonaldist*diagonaldist*diagonaldist*diagonaldist;
        }



        if (gamepad2.right_trigger > 0.5 && (intaekstage == -1 || intaekstage == 20)) {
            intaekstage = 5;
            intakeeee.reset();
        }



        if (gamepad1.x) {
            settherotation(spina + 0.5);
            settherotation(spina);
        }


        if (gamepad1.a) {
            settherotation(spina); //default intake pos
        }



        switch (intaekstage) {
            case 5:
                if (intakeeee.time() > 0.0005) {
                    headingLock = true;
                    setrightdown();
                    intaekstage = 6;
                    intakeeee.reset();}
                break;
            case 6:
                settherotation(spina);
                if (!follower.isBusy()) {
                    intaekstage = 7;
                    intakeeee.reset();}
                break;
            case 7:
                if (holdshooting) {
                    parksettherotation(0.1);
                }
                spinflickup();
                if (intakeeee.time() > 0.056) {
                    intaekstage = 8;
                    intakeeee.reset();}
                break;
            case 8:
                spinflickdown();
                if (intakeeee.time() > 0.056) {
                    intaekstage = 9;
                    intakeeee.reset();}
                break;
            case 9:
                settherotation(spinb);
                if (intakeeee.time() > 0.56) {
                    intaekstage = 10;
                    intakeeee.reset();}
                break;
            case 10:
                spinflickup();
                if (intakeeee.time() > 0.056) {
                    intaekstage = 11;
                    intakeeee.reset();}
                break;
            case 11:
                spinflickdown();
                if (intakeeee.time() > 0.056) {
                    intaekstage = 12;
                    intakeeee.reset();}
                break;
            case 12:
                settherotation(spinc);
                if (intakeeee.time() > 0.57) {
                    intaekstage = 13;
                    intakeeee.reset();}
                break;
            case 13:
                spinflickup();
                if (intakeeee.time() > 0.65) {
                    headingLock = false;
                    intaekstage = 14;
                    intakeeee.reset();}
                break;
            case 14:
                parksettherotation(0);
                headingLock = false;
                spinflickdown();
                if (intakeeee.time() > 0.056) {
                    intaekstage = -1;
                    intakeeee.reset();
                    settherotation(spina);}
                break;

        }

        if (gamepad1.rightStickButtonWasPressed()) {
            follower.followPath(pathChain);
        }
        if (gamepad1.rightStickButtonWasReleased()) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            slowMode = true;
            headingLock = false;
        }

        if (gamepad1.dpad_left) {
            follower.setPose(new Pose(72,9,(Math.toRadians(90))));
        }


        if (gamepad1.left_stick_button) {
            intaekstage = -1;
            headingLock = false;
            spinflickdown();
            parksettherotation(0);
            settherotation(spina);
        }

//        if (gamepad2.dpad_up) {
//            flickright.setPosition(flickup);
//            flickys.setPosition(flickup);
//        }


//        if (gamepad1.right_trigger > 0.2 && !intakepressed) {
//            intakepressed = true;
//            if (intaketoggle) {
//                intaketoggle = false;
//            } else if (!intaketoggle) {
//                intaketoggle = true;
//            }
//        }
//        if (gamepad1.right_trigger < 0.2) {
//            intakepressed = false;
//        }

        if (gamepad1.right_bumper) {
            intake.setPower(1);
        }
        if (gamepad1.right_trigger > 0.25) {
            intake.setPower(0);
        }
        if (gamepad1.left_trigger > 0.25) {
            intake.setPower(-1);
        }

//        if (intaketoggle) {
//            intake.setPower(-1);
//        }
//        if (!intaketoggle) {
//            intake.setPower(0);
//        }

        if (gamepad1.left_bumper) {
            headingLock = true;
        }

        if (gamepad1.dpad_right) {
            slowMode = false;
        }


//        if (gamepad1.dpad_down) {
//            holdshooting = true;
//        }
//        if (gamepad1.dpad_up) {
//            holdshooting = false;
//            parksettherotation(0);
//        }

        if (gamepad2.dpad_down) {
            settherotation(spina);


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


            llresultt = getPatternIdAuto();

            if (llresultt == 21 && PP) { //GPP
                thedefault = 4;
                dononee = true;
                skiponee = false;
                reverseonee = false;
            }
            else if (llresultt == 21 && PG) { //GPP
                reverse1 = 4;
                dononee = false;
                skiponee = false;
                reverseonee = true;
            }
            else if (llresultt == 21 && GP) { //GPP
                skip1 = 4;
                dononee = false;
                skiponee = true;
                reverseonee = false;
            }
            else if (llresultt == 22 && PP) { //PGP
                reverse1 = 4;
                dononee = false;
                skiponee = false;
                reverseonee = true;
            }
            else if (llresultt == 22 && PG) { //PGP
                skip1 = 4;
                dononee = false;
                skiponee = true;
                reverseonee = false;
            }
            else if (llresultt == 22 && GP) { //PGP
                thedefault = 4;
                dononee = true;
                skiponee = false;
                reverseonee = false;
            }
            else if (llresultt == 23 && PP) { //PPG
                skip1 = 4;
                dononee = false;
                skiponee = true;
                reverseonee = false;
            }
            else if (llresultt == 23 && PG) { //PPG
                thedefault = 4;
                dononee = true;
                skiponee = false;
                reverseonee = false;
            }
            else if (llresultt == 23 && GP) { //PPG
                reverse1 = 4;
                dononee = false;
                skiponee = false;
                reverseonee = true;
            }
            else {
                thedefault = 4;
            }
        }

        sortone();
        sorttwo();
        sortthree();

        if (gamepad1.dpad_down) {
            if (skiponee) {
                skip1 = 5;
            }
            if (dononee) {
                thedefault = 5;
            }
            if (reverseonee) {
                reverse1 = 5;
            }
        }

        turnerror = headinglockangle - Math.toDegrees(botHeading);
        controller.updateError(turnerror);

        if (headingLock)
            follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, controller.run(), true);
//        else if (slowMode)
//            follower.setTeleOpDrive(-gamepad2.left_stick_y*0.5, -gamepad2.left_stick_x*0.5, -gamepad2.right_stick_x*0.5, true);
        else
            follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x, true);


        if (gamepad1.y) {
            parksettherotation(0);
        }
        if (gamepad1.b) {
            parksettherotation(1);


            telemetry.addData("diag dist", diagonaldist);
            telemetry.addData("error", turnerror);
            telemetry.addData("targetV", targetV);
            telemetry.addData("actualV", sencoder.getVelocity());
            telemetry.addData("intake stage", intaekstage);
            telemetry.addData("timer", intaketimercount);
            telemetry.addData("headinglockangle", headinglockangle);
            telemetry.addData("llresult", llresultt);
            telemetry.update();


        }

    }}


/// OPERATOR CONTROLS AS FOLLOWS:

/// left trigger: intake
/// left bumper: enable heading lock
/// right trigger: intake off
/// right bumper: outtake

/// dpad up: shoot sort
/// dpad down: get id, prepare for sort

/// dpad left: reset pose

/// dpad right: slowmode off
/// a: reset spindexer plate
/// b: park, turn flywheel off if held
/// x: shake spindexer
/// y: retract park

/// left stick button: reset shooting cycle

///right stick button: hold to auto park, release to manual drive


///ADD SLOWMODE TO BLUE