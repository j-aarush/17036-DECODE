package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.function.Supplier;



@TeleOp(name = "PEDROTELEOPBLUE")
@Configurable
public class PEDROTELEOPBLUE extends NextFTCOpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(50.5, 25.0, Math.toRadians(108.0)).mirror();
    private final Pose scorePose = new Pose(60, 14, Math.toRadians(110.57)).mirror(); //figure outt

    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    public static Servo leftspindex, rightspindex;

    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    public static DcMotorEx intake, flywheel;
    public static float targetV = 0;

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


    int intaekstage = -1, shooterstage = -1, previntakestage = -1;



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


        flickys.setPosition(flickup);
        flickys.setPosition(flickdown);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, follower.getPose())))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, scorePose.getHeading(), 0.8))
                .build();

    }

    @Override
    public void onStartButtonPressed() {
        intakeeee.reset();
        follower.startTeleopDrive();
    }

    @Override
    public void onUpdate() {
        follower.update();

        error = targetV - flywheel.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);

        if (gamepad1.y) {
            targetV = 0;
        }
        if (gamepad1.x) {
            targetV = 1200;
        }
        if (gamepad1.b) {
            targetV = 1550;
        }

        telemetry.addData("targetV", targetV);
        telemetry.addData("velocity", flywheel.getVelocity());
        telemetry.update();

//            shootingfsmbutton.whenTrue(() -> shootingfsm());

        if (gamepad2.right_trigger > 0.5 && (intaekstage == -1 || intaekstage == 20)) {
            intaekstage = 5;
            intakeeee.reset();
        }


        if (gamepad1.left_bumper) {
            intake.setPower(-1);
        }
        if (gamepad1.right_bumper) {
            intake.setPower(1);
        }
        if (gamepad1.right_trigger > 0.5) {
            intake.setPower(0);
        }



        if (gamepad1.a) {
            settherotation(0.355); //default intake pos

        }




        switch (intaekstage) {
            case 5:
                previntakestage = 5;
                if (intakeeee.time() > 0.025) {
                    intaekstage = 6;
                    intakeeee.reset();}
                break;
            case 6:
                settherotation(0.355);
                previntakestage = 6;
                if (intakeeee.time() > 0.1) {
                    intaekstage = 7;
                    intakeeee.reset();}
                break;
            case 7:
                flickys.setPosition(flickup); //hopefully up
                previntakestage = 7;
                if (intakeeee.time() > 0.25) {
                    intaekstage = 8;
                    intakeeee.reset();}
                break;
            case 8:
                flickys.setPosition(flickdown); //hopefully up
                previntakestage = 8;
                if (intakeeee.time() > 0.25) {
                    intaekstage = 9;
                    intakeeee.reset();}
                break;
            case 9:
                rotationpos = rotationpos - 0.255;
                settherotation(0.61);
                previntakestage = 9;
                if (intakeeee.time() > 0.7) {
                    intaekstage = 10;
                    intakeeee.reset();}
                break;
            case 10:
                flickys.setPosition(flickup); //hopefully up
                previntakestage = 10;
                if (intakeeee.time() > 0.25) {
                    intaekstage = 11;
                    intakeeee.reset();}
                break;
            case 11:
                flickys.setPosition(flickdown); //hopefully down
                previntakestage = 11;
                if (intakeeee.time() > 0.25) {
                    intaekstage = 12;
                    intakeeee.reset();}
                break;
            case 12:
                rotationpos = rotationpos - 0.255;
                settherotation(0.865);
                previntakestage = 9;
                if (intakeeee.time() > 0.7) {
                    intaekstage = 13;
                    intakeeee.reset();}
                break;
            case 13:
                flickys.setPosition(flickup); //hopefully up
                previntakestage = 10;
                if (intakeeee.time() > 0.25) {
                    intaekstage = 14;
                    intakeeee.reset();}
                break;
            case 14:
                flickys.setPosition(flickdown); //hopefully down
                previntakestage = 11;
                if (intakeeee.time() > 0.25) {
                    intaekstage = -1;
                    intakeeee.reset();
                    settherotation(0.355);}
                break;

        }



//        if (gamepad1.dpad_right) {
//            settherotation(0.5);
//        }
//        if (gamepad1.dpad_left) {
//            settherotation(0);
//        }
        if (gamepad1.dpad_right) {
            flickys.setPosition(flickup);
        }
        if (gamepad1.dpad_left) {
            flickys.setPosition(flickdown);
        }


        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    -gamepad2.left_stick_y,
                    -gamepad2.left_stick_x,
                    -gamepad2.right_stick_x,
                    true // Robot Centric
            );
            //This is how it looks with slowMode on
        }
        //Automated PathFollowing
        if (gamepad1.dpadUpWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.dpadDownWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }




        telemetry.addData("intake stage", intaekstage);
        telemetry.addData("timer", intaketimercount);
        telemetry.update();
//claire skibidi toilet ohio



    }

}
//coding todos (for later):
//set up pinpoint/pedro
//format as much code as possible into seperate subsystem classes
//heading lock via ll/pp
//indexing
//formula for flywheel speed


//driver controls: field centric joysticks (objectively better than robot centric), shooter mode on/off (HOLD to keep shooting, not just press)
//opperator controls: *heading lock on/off*, shooter speed, intake mode on/off, intake motor on/off
//MONDAY -- FINISH TELEOP BY ADDING HEADING LOCK, TUNE PEDRO (SO WE CAN HAVE A BASIC AUTO)