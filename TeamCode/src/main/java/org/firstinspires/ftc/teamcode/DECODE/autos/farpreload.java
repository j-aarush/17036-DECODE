package org.firstinspires.ftc.teamcode.DECODE.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;

import static dev.nextftc.bindings.Bindings.button;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "3 Ball Far", preselectTeleOp = "NEWnewnewnew TELEOP")
@Configurable
public class farpreload extends NextFTCOpMode {


    public static double flywheelvelocity;


    public static MotorEx flywheel = new MotorEx("shooter");


    public static float configvelocity = 800; //far zone - ~1500. near zone - ~1200-1300


    public static NormalizedColorSensor colorSensor;
    public static Servo leftspindex, rightspindex;

    public static void velocityControlWithFeedforwardExample(KineticState currentstate) {
        // Create a velocity controller with PID and feedforward
        ControlSystem controller = ControlSystem.builder()
                .velPid(0.184, 0.02, 0.05) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(0.0067, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        // Set the goal velocity to 500 units per second
        controller.setGoal(new KineticState(0.0, 1000, 0.0));

        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity
        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity

        double power = controller.calculate(currentstate);
        flywheel.setPower(power * .5);

        // Apply power to your motor
        System.out.println("Power to apply: " + power);
    }

    public static void shooter() {
        BindingManager.update();
        flywheelvelocity = flywheel.getVelocity();
        KineticState currentState = new KineticState(0, 1000, 0.0); //figure out velocity (is it in ticks?!?)
        velocityControlWithFeedforwardExample(currentState);


    }

        public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    public static DcMotorEx intake;
    float greenv, bluev, redv;
    double flickup = 0.00, flickdown = 0.3;
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

    public void driveawatt() {
        FL.setPower(1);
        BL.setPower(1);
        FR.setPower(-1);
        BR.setPower(-1);
    }
    public void stopd() {
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }


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
        settherotation(0.45); //first pos

    }

    @Override
    public void onUpdate() {

        telemetry.update();


//claire skibidi toilet ohio



    }


    @Override
    public void onStartButtonPressed() {

        shooter();

        telemetry.addData("output real velocity:", flywheelvelocity);
        telemetry.addData("input velocity:", configvelocity);
        sleep(3000);

        settherotation(0.45); //first pos
        sleep(1500);
        flickys.setPosition(flickup); //hopefully up
        sleep(2000);

        flickys.setPosition(flickdown); //hopefully down
        sleep(1500);

        settherotation(0.7);
        sleep(1000);

        flickys.setPosition(flickup); //hopefully up
        sleep(2000);

        flickys.setPosition(flickdown); //hopefully up
        sleep(1500);

        settherotation(0.954);
        sleep(1000);

        flickys.setPosition(flickup); //hopefully up
        sleep(1000);

        flickys.setPosition(flickdown); //hopefully down
        sleep(1000);

        settherotation(0.325);
        sleep(100);

        driveawatt();
        sleep(350);
        stopd();
        stop();




    }


}