package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.FilteredPIDFController;
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
    private PathChain pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    public static Servo leftspindex, rightspindex;

    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    public static DcMotorEx intake, flywheel;
    public static double targetV = 0;
    FilteredPIDFController controller = new FilteredPIDFController(new FilteredPIDFCoefficients(1,0,0,0,0 ));

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

//    PIDFController controller = new PIDFController(follower.constants.coefficientsHeadingPIDF)
//            .setCoefficientsHeadingPIDF(new PIDFCoefficients(0,0,0,0)
            ;



    boolean headingLock = true;

    int intaekstage = -1, shooterstage = -1, previntakestage = -1;
    boolean heaaidnglock = false;
    double headinglockangle;



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
        follower.setStartingPose(new Pose(50.5, 25.0, Math.toRadians(108.0)));
        follower.update();
//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, follower.getPose())))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, scorePose.getHeading(), 0.8))
//                .build();

    }

    @Override
    public void onStartButtonPressed() {
        intakeeee.reset();
        follower.startTeleopDrive();
        follower.update();

    }

    @Override
    public void onUpdate() {





        error = targetV - flywheel.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);

        if (gamepad1.y) {
            targetV = 0;
        }
        if (gamepad1.x) {
            targetV = 1100;
        }
        if (gamepad1.b) {
            targetV += 1;
        }
        if (gamepad1.a) {
            targetV -= 1;
        }
        if (gamepad1.dpad_left) flickys.setPosition(flickdown);
        if (gamepad1.dpad_right) flickys.setPosition(flickup);

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
            settherotation(0.372); //default intake pos

        }




        switch (intaekstage) {
            case 5:
                previntakestage = 5;
                if (intakeeee.time() > 0.025) {
                    intaekstage = 6;
                    intakeeee.reset();}
                break;
            case 6:
                settherotation(0.372);
                previntakestage = 6;
                if (intakeeee.time() > 0.1) {
                    intaekstage = 7;
                    intakeeee.reset();}
                break;
            case 7:
                flickys.setPosition(flickup); //hopefully up
                previntakestage = 7;
                if (intakeeee.time() > 0.08) {
                    intaekstage = 8;
                    intakeeee.reset();}
                break;
            case 8:
                flickys.setPosition(flickdown); //hopefully up
                previntakestage = 8;
                if (intakeeee.time() > 0.08) {
                    intaekstage = 9;
                    intakeeee.reset();}
                break;
            case 9:
                rotationpos = rotationpos - 0.255;
                settherotation(0.622);
                previntakestage = 9;
                if (intakeeee.time() > 0.7) {
                    intaekstage = 10;
                    intakeeee.reset();}
                break;
            case 10:
                flickys.setPosition(flickup); //hopefully up
                previntakestage = 10;
                if (intakeeee.time() > 0.08) {
                    intaekstage = 11;
                    intakeeee.reset();}
                break;
            case 11:
                flickys.setPosition(flickdown); //hopefully down
                previntakestage = 11;
                if (intakeeee.time() > 0.08) {
                    intaekstage = 12;
                    intakeeee.reset();}
                break;
            case 12:
                rotationpos = rotationpos - 0.255;
                settherotation(0.877);
                previntakestage = 9;
                if (intakeeee.time() > 0.7) {
                    intaekstage = 13;
                    intakeeee.reset();}
                break;
            case 13:
                flickys.setPosition(flickup); //hopefully up
                previntakestage = 10;
                if (intakeeee.time() > 0.08) {
                    intaekstage = 14;
                    intakeeee.reset();}
                break;
            case 14:
                flickys.setPosition(flickdown); //hopefully down
                previntakestage = 11;
                if (intakeeee.time() > 0.08) {
                    intaekstage = -1;
                    intakeeee.reset();
                    settherotation(0.372);}
                break;

        }



//        if (gamepad1.dpad_right) {
//            settherotation(0.5);
//        }
//        if (gamepad1.dpad_left) {
//            settherotation(0);
//        }
//        if (gamepad1.dpad_right) {
//            flickys.setPosition(flickup);
//        }
//        if (gamepad1.dpad_left) {
//            flickys.setPosition(flickdown);
//        }


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
//        if (gamepad1.dpadUpWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.dpadDownWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }


        double posx = follower.getPose().getX();
        double posy = follower.getPose().getY();
        double distx = posx - 10;
        double disty = Math.abs(137 - posy);
        double diagonaldist = Math.sqrt(distx*distx + disty*disty);
        double trigangle = Math.toDegrees(Math.atan(disty/distx));
        headinglockangle = 90 - trigangle + 90;


            if (gamepad1.left_trigger > 0.1) {
                headingLock = true;
            } else headingLock = false;

            turnerror = headinglockangle - follower.getHeading();
            controller.updateError(turnerror);

            if (headingLock)
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, controller.run(), true);
            else
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);


        telemetry.addData("diag dist", diagonaldist);

        error = targetV - flywheel.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);

        targetV = 2447 + -51.2*diagonaldist + 0.753*diagonaldist*diagonaldist + -0.00437*diagonaldist*diagonaldist*diagonaldist + 0.0000091*diagonaldist*diagonaldist*diagonaldist*diagonaldist;

        telemetry.addData("intake stage", intaekstage);
        telemetry.addData("timer", intaketimercount);
        telemetry.addData("posx", posx);
        telemetry.addData("posy", posy);
        telemetry.addData("distx", distx);
        telemetry.addData("heading", headingLock);
        telemetry.addData("disty", disty);
//        telemetry.addData("trigangle", trigangle);
        telemetry.addData("headinglockangle", headinglockangle);
        telemetry.update();



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