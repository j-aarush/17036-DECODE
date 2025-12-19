package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.DECODE.autos.sixspecautooored;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.function.Supplier;




@TeleOp(name = "PEDROTELEOPRED")
@Configurable
public class PEDROTELEOPRED extends NextFTCOpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(50.5, 25.0, Math.toRadians(108.0));
    private final Pose scorePose = new Pose(60, 14, Math.toRadians(110.57)); //figure outt
    private boolean automatedDrive;
    private PathChain pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    double headinglockangle;

    double spina = 0.24;
    double spinb = 0.495;
    double spinc = 0.75;
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

    public static DcMotorEx intake, flywheel;
    public static double targetV = 0;
    boolean heaaidnglock = false;

    double kP = 0.11, kV = 0.000435;
    double error;

    float greenv, bluev, redv;
    double flickup = 0.0, flickdown = 0.5;
    double distancev;
    boolean move = false, intakeonoffb = false;
    boolean intakeswitch = false;
    int counter = 0;
    int shootercounter = 0;
    double rotationpos;

    double turnerror;
    public static Servo leftpark, rightpark;

    public void parksettherotation(double rotationn) {
        leftpark.setPosition(rotationn);
        rightpark.setPosition(rotationn);
    }
    PIDFController controller = new PIDFController(new PIDFCoefficients(0.1,0,0.006,0.000004));


    int intaekstage = -1, shooterstage = -1, previntakestage = -1;


    boolean headingLock;

    DcMotorEx FL, FR, BL, BR, leftinake, rightinake;
    Servo flickys;

    double botHeading;
    public static ElapsedTime intakeeee = new ElapsedTime(0);

    public static ElapsedTime getIntakeeee() {
        return intakeeee;
    }

    double intaketimercount;

    @Override
    public void onInit() {

        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "BR");
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flickys = hardwareMap.get(Servo.class, "flicky");
        flickys.setDirection(Servo.Direction.FORWARD);

        leftspindex = hardwareMap.get(Servo.class, "leftspindex");
        rightspindex = hardwareMap.get(Servo.class, "rightspindex");

        intake = hardwareMap.get(DcMotorEx.class, "Lintake");

        leftpark = hardwareMap.get(Servo.class, "leftpark");
        rightpark = hardwareMap.get(Servo.class, "rightpark");
        leftpark.setDirection(Servo.Direction.FORWARD);
        rightpark.setDirection(Servo.Direction.REVERSE);


        flickys.setPosition(flickup);
        flickys.setPosition(flickdown);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(50.5, 25.0, Math.toRadians(108.0)).mirror());
        follower.update();
        headingLock = false;

    }

    @Override
    public void onStartButtonPressed() {
        intakeeee.reset();
        follower.startTeleopDrive();
        follower.update();
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


        targetV = 2447 + -51.2*diagonaldist + 0.753*diagonaldist*diagonaldist + -0.00437*diagonaldist*diagonaldist*diagonaldist + 0.0000091*diagonaldist*diagonaldist*diagonaldist*diagonaldist;

        error = targetV - flywheel.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);


        if (gamepad1.y) {
            targetV = 0;
        }

        if (gamepad1.dpad_left) flickys.setPosition(flickdown);
        if (gamepad1.dpad_right) flickys.setPosition(flickup);



        if (gamepad2.right_trigger > 0.5 && (intaekstage == -1 || intaekstage == 20)) {
            intaekstage = 5;
            intakeeee.reset();
        }


        if (gamepad1.left_trigger > 0.5) {
            intake.setPower(-1);
        }
        if (gamepad1.right_bumper) {
            intake.setPower(1);
        }
        if (gamepad1.right_trigger > 0.5) {
            intake.setPower(0);
        }



        if (gamepad1.a) {
            settherotation(spina); //default intake pos
        }



        switch (intaekstage) {
            case 5:
                if (intakeeee.time() > 0.025) {
                    headingLock = true;
                    intaekstage = 6;
                    intakeeee.reset();}
                break;
            case 6:
                settherotation(spina);
                if (intakeeee.time() > 0.1) {
                    intaekstage = 7;
                    intakeeee.reset();}
                break;
            case 7:
                flickys.setPosition(flickup); //hopefully up
                if (intakeeee.time() > 0.07) {
                    intaekstage = 8;
                    intakeeee.reset();}
                break;
            case 8:
                flickys.setPosition(flickdown); //hopefully up
                if (intakeeee.time() > 0.07) {
                    intaekstage = 9;
                    intakeeee.reset();}
                break;
            case 9:
                rotationpos = rotationpos - 0.255;
                settherotation(spinb);
                if (intakeeee.time() > 0.75) {
                    intaekstage = 10;
                    intakeeee.reset();}
                break;
            case 10:
                flickys.setPosition(flickup); //hopefully up
                if (intakeeee.time() > 0.07) {
                    intaekstage = 11;
                    intakeeee.reset();}
                break;
            case 11:
                flickys.setPosition(flickdown); //hopefully down
                if (intakeeee.time() > 0.07) {
                    intaekstage = 12;
                    intakeeee.reset();}
                break;
            case 12:
                settherotation(spinc);
                if (intakeeee.time() > 0.75) {
                    intaekstage = 13;
                    intakeeee.reset();}
                break;
            case 13:
                flickys.setPosition(flickup); //hopefully up
                if (intakeeee.time() > 0.07) {
                    headingLock = false;
                    intaekstage = 14;
                    intakeeee.reset();}
                break;
            case 14:
                flickys.setPosition(flickdown); //hopefully down
                headingLock = false;
                if (intakeeee.time() > 0.07) {
                    intaekstage = -1;
                    intakeeee.reset();
                    settherotation(spina);}
                break;

        }


        if (gamepad1.left_bumper && !lefttoggle) {
            lefttoggle = true;
            if (headingLock) {
                headingLock = false;
            } else if (!headingLock) {
                headingLock = true;
            }
        }
        if (!gamepad1.left_bumper) {
            lefttoggle = false;
        }

        turnerror = headinglockangle - Math.toDegrees(botHeading);
        controller.updateError(turnerror);

        if (headingLock)
            follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, controller.run(), true);
        else
            follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x, true);


        if (gamepad1.y) {
            parksettherotation(0);
        }
        if (gamepad1.b) {
            parksettherotation(0.75);


            telemetry.addData("diag dist", diagonaldist);
            telemetry.addData("error", turnerror);
            telemetry.addData("current heading", follower.getHeading());
            telemetry.addData("targetV", targetV);
            telemetry.addData("actualV", flywheel.getVelocity());
            telemetry.addData("intake stage", intaekstage);
            telemetry.addData("timer", intaketimercount);
            telemetry.addData("headinglockangle", headinglockangle);
            telemetry.update();


        }

}}
//coding todos (for later):
//set up pinpoint/pedro
//format as much code as possible into seperate subsystem classes
//heading lock via ll/pp
//indexing
//formula for flywheel speed


//driver controls: field centric joysticks (objectively better than robot centric), shooter mode on/off (HOLD to keep shooting, not just press)
//opperator controls: *heading lock on/off*, shooter speed, intake mode on/off, intake motor on/off
//MONDAY -- FINISH TELEOP BY ADDING HEADING LOCK, TUNE PEDRO (SO WE CAN HAVE A BASIC AUTO)