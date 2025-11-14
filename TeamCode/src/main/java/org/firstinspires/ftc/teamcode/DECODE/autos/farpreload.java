package org.firstinspires.ftc.teamcode.DECODE.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import dev.nextftc.bindings.Button;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.NextFTCOpMode;
import static org.firstinspires.ftc.teamcode.DECODE.PIDs.flywheelpid.*;
import static dev.nextftc.bindings.Bindings.button;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "3 Ball Far")
@Disabled
@Configurable
public class farpreload extends NextFTCOpMode {
    public static NormalizedColorSensor colorSensor;
    public static Servo leftspindex, rightspindex;

    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    public static DcMotorEx intake;
    float greenv, bluev, redv;
    double flickup = 0.00, flickdown = 0.267;
    double distancev;
    boolean move = false, intakeonoffb = false;
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

    GoBildaPinpointDriver pinpoint;


    Button gamepad1y = button(() -> gamepad1.y);
    Button gamepad1b = button(() -> gamepad1.b);
    Button gamepad1x = button(() -> gamepad1.x);
    Button shootingfsmbutton = button(() -> gamepad2.right_bumper);
    Button intakeonoff = button(() -> gamepad1.right_bumper);


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


        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flickys = hardwareMap.get(Servo.class, "flicky");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakecolor");
        leftspindex = hardwareMap.get(Servo.class, "leftspindex");
        rightspindex = hardwareMap.get(Servo.class, "rightspindex");
        intake = hardwareMap.get(DcMotorEx.class, "Lintake");


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setHeading(0, AngleUnit.DEGREES);

        // Configure the sensor
        colorSensor.setGain(100);
    }

    @Override
    public void onStartButtonPressed() {
        intakeeee.reset();
        configvelocity = 1600;

        settherotation(0.62); //first pos
        flickys.setPosition(flickup); //hopefully up
        flickys.setPosition(flickdown); //hopefully down
        settherotation(0.872);
        flickys.setPosition(flickup); //hopefully up
        flickys.setPosition(flickdown); //hopefully up
        settherotation(0.368);
        flickys.setPosition(flickup); //hopefully up
        flickys.setPosition(flickdown); //hopefully down
        settherotation(0);



    }

    @Override
    public void onUpdate() {

        shooter();

        telemetry.addData("output real velocity:", flywheelvelocity);
        telemetry.addData("input velocity:", configvelocity);
        telemetry.update();
//claire skibidi toilet ohio



    }

}