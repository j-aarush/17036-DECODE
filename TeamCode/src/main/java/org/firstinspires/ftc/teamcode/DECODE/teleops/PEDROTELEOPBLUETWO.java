package org.firstinspires.ftc.teamcode.DECODE.teleops;

import static org.firstinspires.ftc.teamcode.DECODE.botconstants.autoendpose;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.flickdown;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.flickup;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spina;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinb;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinc;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
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
import com.pedropathing.paths.PathChain;


@TeleOp(name = "BLUE TELEOP 2")
@Configurable
public class PEDROTELEOPBLUETWO extends NextFTCOpMode {
    private Follower follower;
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
        sencoder = hardwareMap.get(DcMotorEx.class, "sencoder");
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
//        follower.setStartingPose(new Pose(50.5, 25.0, Math.toRadians(108.0)).mirror());
        follower.setStartingPose(autoendpose);
        follower.update();
        headingLock = false;

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, new Pose(110,35)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(270), 0.8))
                .build();
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
        posx = follower.getPose().getX();
        posy = follower.getPose().getY();
        distx = posx - 8;
        disty = Math.abs(137 - posy);
        diagonaldist = Math.sqrt(distx * distx + disty * disty);
        trigangle = Math.toDegrees(Math.atan(disty / distx));
        headinglockangle = 90 - trigangle + 90;



        error = targetV - sencoder.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);


        if (gamepad1.b) {
            targetV = 0;
        } else {
            targetV = 2447 + -51.2*diagonaldist + 0.753*diagonaldist*diagonaldist + -0.00437*diagonaldist*diagonaldist*diagonaldist + 0.0000091*diagonaldist*diagonaldist*diagonaldist*diagonaldist;
        }

//        if (gamepad1.dpad_left) flickys.setPosition(flickdown);
//        if (gamepad1.dpad_right) flickys.setPosition(flickup);
//


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
                settherotation(spinb);
                if (intakeeee.time() > 0.56) {
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
                if (intakeeee.time() > 0.57) {
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
                parksettherotation(0);
                headingLock = false;
                flickys.setPosition(flickdown); //hopefully down
                if (intakeeee.time() > 0.07) {
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
            flickys.setPosition(flickdown);
            parksettherotation(0);
            settherotation(spina);
        }


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


        if (gamepad1.dpad_down) {
            holdshooting = true;
        }
        if (gamepad1.dpad_up) {
            holdshooting = false;
            parksettherotation(0);
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
            parksettherotation(0.9);


            telemetry.addData("diag dist", diagonaldist);
            telemetry.addData("error", turnerror);
            telemetry.addData("targetV", targetV);
            telemetry.addData("actualV", sencoder.getVelocity());
            telemetry.addData("intake stage", intaekstage);
            telemetry.addData("timer", intaketimercount);
            telemetry.addData("headinglockangle", headinglockangle);
            telemetry.update();


        }

    }}


/// OPERATOR CONTROLS AS FOLLOWS:

/// left trigger: ------
/// left bumper: enable heading lock
/// right trigger: toggle intake off / outtake
/// right bumper: intake in

/// dpad up: enable defense brakes
/// dpad down: disable defense brakes

/// dpad left: reset pose

/// dpad right: slowmode off
/// a: reset spindexer plate
/// b: park, turn flywheel off if held
/// x: shake spindexer
/// y: retract park

/// left stick button: reset shooting cycle

///right stick button: hold to auto park, release to manual drive


///ADD SLOWMODE TO BLUE