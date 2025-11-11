package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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


@TeleOp(name = "robot centric")
@Configurable
public class robotcentricteleop extends NextFTCOpMode {
    public static  NormalizedColorSensor colorSensor;
    public static ElapsedTime intakeeee = new ElapsedTime();
    public static Servo leftspindex, rightspindex;
    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    public static DcMotorEx intake;
    float greenv, bluev;
    boolean move= false;
    int counter = 0;
    int shootercounter = 0;
    double rotationpos;

    int intaekstage =-1 , shooterstage =-1;

    double y;
    double x ;
    double rx ;

    // Denominator is the largest motor power (absolute value) or 1
    // This ensures all the powers maintain the same ratio,
    // but only if at least one is out of the range [-1, 1]
    double denominator ;
    double frontLeftPower;
    double backLeftPower ;
    double frontRightPower ;
    double backRightPower ;

    GoBildaPinpointDriver pinpoint;


    Button gamepad1y = button(() -> gamepad1.y);
    Button gamepad1b = button(() -> gamepad1.b);
    Button gamepad1x = button(() -> gamepad1.x);
    Button shootingfsmbutton = button(() -> gamepad2.right_bumper);
    Button intakeonoff = button(() -> gamepad1.right_bumper);


    public void intakefsm() {
    }
    public void shootingfsm() {
        switch (shooterstage) {
            case 0:
                rotationpos = 0.6167;
                settherotation(0.6167); //first pos
                shooterstage = 1;
                break;
            case 1:
                flicky.setPosition(0); //hopefully up
                shooterstage = 2;
                break;
            case 2:
                flicky.setPosition(0.25); //hopefully down
                shooterstage = 3;
                break;
            case 3:
                rotationpos = rotationpos - 0.255;
                settherotation(rotationpos);
                shooterstage = 4;
                break;
            case 4:
                flicky.setPosition(0); //hopefully up
                shooterstage = 5;
                break;
            case 5:
                flicky.setPosition(0.25); //hopefully up
                shooterstage = 6;
                break;
            case 6:
//                    rotationpos = 0.175;
//                    settherotation(rotationpos); //first pos
                rotationpos = rotationpos - 0.2535;
                settherotation(rotationpos);
                shooterstage = 7;
                break;
            case 7:
                flicky.setPosition(0); //hopefully up
                shooterstage = 8;
                break;
            case 8:
                flicky.setPosition(0.25); //hopefully down
                settherotation(0);
                shooterstage = -1;
                break;
        }
    }


    DcMotorEx FL, FR, BL, BR, leftinake, rightinake;
    Servo flicky;

    double botHeading;


    @Override
    public void runOpMode() {



        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flicky = hardwareMap.get(Servo.class, "flicky");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakecolor");
        leftspindex = hardwareMap.get(Servo.class, "leftspindex");
        rightspindex = hardwareMap.get(Servo.class, "rightspindex");
        intake = hardwareMap.get(DcMotorEx.class, "Lintake");


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setHeading(0, AngleUnit.DEGREES);

        // Configure the sensor


        waitForStart();
        colorSensor.setGain(12);

        while(opModeIsActive()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            greenv = colors.green;
            bluev = colors.blue;
//            pinpoint.update();
//            botHeading = pinpoint.getHeading(AngleUnit.DEGREES);



            if (greenv > 0.0167 && greenv < 0.04 && bluev > 0.015 && bluev < 0.039) {
                move = true;
                intakeeee.startTime();
            } else {
                move = false;
            }


            //get data from hub; store as variables at beginning of loop

            gamepad1y.whenBecomesTrue(() -> configvelocity = 1267); //rough near zone

            gamepad1b.whenBecomesTrue(() -> configvelocity = 1520); //rough far zone

            gamepad1x.whenBecomesTrue(() -> configvelocity = 500); //power save

//            shootingfsmbutton.whenTrue(() -> shootingfsm());


            if (gamepad1.left_bumper) {
                intaekstage = 0;
            }

            intakeonoff.toggleOnBecomesTrue()
                    .whenBecomesTrue(() -> intake.setPower(0)) // runs every other rising edge, including the first one
                    .whenBecomesFalse(() -> intake.setPower(1)); // runs the rest of the rising edges


            switch (intaekstage) {
                case 0:
                    intake.setPower(1);
//                    rotationpos = 0;
                    settherotation(0); //first pos
                    if (move && intakeeee.seconds() > 0.467) {
//                        rotationpos = rotationpos + 0.25;
                        settherotation(0.25);
                        move = false;
                        intakeeee.reset();
                        intaekstage = 1;
                    }
                    break;
                case 1:
                    settherotation(0.25); //first pos
                    if (move && intakeeee.seconds() > 0.467) {
//                        rotationpos = rotationpos + 0.25;
                        settherotation(0.5);
                        move = false;
                        intakeeee.reset();
                        intaekstage = 2;
                    }
                    break;
                case 2:
//                rotationpos = 0.175;
                    settherotation(0.5);
//                    settherotation(rotationpos); //first pos
                    if (move && intakeeee.seconds() > 0.467) {
                        settherotation(0.6167);
                        move = false;
                        intakeeee.reset();
                        intake.setPower(0);
                        intaekstage = -1;
                        shooterstage = 0;
                    }
                    break;
            }



            if (gamepad1.dpad_right) {
                settherotation(0.5);
            }
            if (gamepad1.dpad_left) {
                settherotation(0);
            }




            double rx = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double y = gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            shooter();


            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

            telemetry.addData("output real velocity:", flywheelvelocity);
            telemetry.addData("input velocity:", configvelocity);
            telemetry.addData("green:", greenv);
            telemetry.addData("blue:", bluev);
            telemetry.update();
//claire skibidi toilet ohio

        }

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