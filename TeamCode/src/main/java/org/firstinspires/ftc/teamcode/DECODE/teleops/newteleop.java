package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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


@TeleOp(name = "NEW TELEOP")
@Configurable
public class newteleop extends NextFTCOpMode {
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


//    public void shootingfsm() {
//        switch (shooterstage) {
//            case 0:
//                rotationpos = 0.6167;
//                settherotation(0.6167); //first pos
//                shooterstage = 1;
//                break;
//            case 1:
//                flicky.setPosition(flickup); //hopefully up
//                shooterstage = 2;
//                break;
//            case 2:
//                flicky.setPosition(flickdown); //hopefully down
//                shooterstage = 3;
//                break;
//            case 3:
//                rotationpos = rotationpos - 0.255;
//                settherotation(rotationpos);
//                shooterstage = 4;
//                break;
//            case 4:
//                flicky.setPosition(flickup); //hopefully up
//                shooterstage = 5;
//                break;
//            case 5:
//                flicky.setPosition(flickdown); //hopefully up
//                shooterstage = 6;
//                break;
//            case 6:
    /// /                    rotationpos = 0.175;
    /// /                    settherotation(rotationpos); //first pos
//                rotationpos = rotationpos - 0.2535;
//                settherotation(rotationpos);
//                shooterstage = 7;
//                break;
//            case 7:
//                flicky.setPosition(flickup); //hopefully up
//                shooterstage = 8;
//                break;
//            case 8:
//                flicky.setPosition(flickdown); //hopefully down
//                settherotation(0);
//                shooterstage = -1;
//                break;
//        }
//    }


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

    }

     @Override
     public void onUpdate() {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            greenv = colors.green;
            bluev = colors.blue;
            redv = colors.red;
            distancev = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            intaketimercount = intakeeee.time();


//            pinpoint.update();
//            botHeading = pinpoint.getHeading(AngleUnit.DEGREES);


            //purple open - 0.17, 0.2, 0.24
            //purplie solid - 0.17, 0.21, 0.3
            //green open - 0.15, 0.32, 0.22
            //gren solid - 0.1, 0.315, 0.230
            //nothing -
            // metal - 0.42, 0.66, 0.49


            if (!move && distancev > 2 && distancev < 6.5 && redv > 0.05 && redv < 0.30 && greenv > 0.10 && greenv < 0.45 && bluev > 0.05 && bluev < 0.40) {
                intakeeee.reset();
                move = true;
            }

            //get data from hub; store as variables at beginning of loop

            if (gamepad1.y) {
                configvelocity = 1267;
            }
            if (gamepad1.b) {
                configvelocity = 1650;
            }


//            shootingfsmbutton.whenTrue(() -> shootingfsm());

            if (gamepad2.right_trigger > 0.5 && (intaekstage == -1 || intaekstage == 20)) {
                intaekstage = 3;
            }
//            if (gamepad2.right_trigger < 0.5 && intaekstage > 2.9) {
//                previntakestage = intaekstage;
//                intaekstage = 20;
//            }



            if (gamepad1.left_bumper) {
                intaekstage = 0;
            }

//            intakeonoff.toggleOnBecomesTrue()
//                    .whenBecomesTrue(() -> intakeonoffb = true) // runs every other rising edge, including the first one
//                    .whenBecomesFalse(() -> intakeonoffb = false); // runs the rest of the rising edges

            if (intakeonoffb) {
                intake.setPower(0.75);
            }
            if (!intakeonoffb) {
                intake.setPower(0);
            }

            if (gamepad1.a)
            {
                settherotation(0.12); //first pos

            }


            switch (intaekstage) {
                case -1:
                    intakeeee.reset();
                    break;
                case 0:
                    settherotation(0.24); //0.12
                    intakeonoffb = true;
//                    rotationpos = 0;
                    if (move && intaketimercount > 1.6) {
//                        rotationpos = rotationpos + 0.25;
                        move = false;
                        intakeeee.reset();
                        settherotation(0.492); //0.37
                        previntakestage = 0;
                        intaekstage = 1;
                    }
                    break;
                case 1:
                    if (move && intaketimercount > 1.6) {
//                        rotationpos = rotationpos + 0.25;
                        move = false;
                        intakeeee.reset();
                        settherotation(0.492); //0.62

                        previntakestage = 1;
                        intaekstage = 2;
                    }
                    break;
                case 2:
//                rotationpos = 0.175;
//                    settherotation(rotationpos); //first pos
                    if (move && intaketimercount > 1.6) {
                        move = false;
                        intakeeee.reset();
                        settherotation(0.75); //0.62
                        intakeonoffb = false;
                        previntakestage = 2;
                        intaekstage = -1;
                        intakeeee.reset();
                    }
                    break;
                case 3:
                    rotationpos = 0.6167;
                    settherotation(0.62); //first pos
                    previntakestage = 3;
                    if (intaketimercount > 0.33) {
                    intaekstage = 4;
                        intakeeee.reset();}
                    break;
                case 4:
                    flickys.setPosition(flickup); //hopefully up
                    previntakestage = 4;
                    if (intaketimercount > 0.33) {
                    intaekstage = 5;
                        intakeeee.reset();}
                    break;
                case 5:
                    flickys.setPosition(flickdown); //hopefully down
                    previntakestage = 5;
                    if (intaketimercount > 0.33) {
                        intaekstage = 6;
                        intakeeee.reset();}
                    break;
                case 6:
                    settherotation(0.872);
                    previntakestage = 6;
                    if (intaketimercount > 0.75) {
                    intaekstage = 7;
                        intakeeee.reset();}
                    break;
                case 7:
                    flickys.setPosition(flickup); //hopefully up
                    previntakestage = 7;
                    if (intaketimercount > 0.33) {
                    intaekstage = 8;
                        intakeeee.reset();}
                    break;
                case 8:
                    flickys.setPosition(flickdown); //hopefully up
                    previntakestage = 8;
                    if (intaketimercount > 0.33) {
                    intaekstage = 9;
                        intakeeee.reset();}
                    break;
                case 9:
                    rotationpos = rotationpos - 0.255;
                    settherotation(0.368);
                    previntakestage = 9;
                    if (intaketimercount > 1) {
                    intaekstage = 10;
                        intakeeee.reset();}
                    break;
                case 10:
                    flickys.setPosition(flickup); //hopefully up
                    previntakestage = 10;
                    if (intaketimercount > 1.2) {
                    intaekstage = 11;
                        intakeeee.reset();}
                    break;
                case 11:
                    flickys.setPosition(flickdown); //hopefully down
                    previntakestage = 11;
                    if (intaketimercount > 0.5) {
                    intaekstage = -1;
                        intakeeee.reset();}
                    break;

            }



            if (gamepad1.dpad_right) {
                settherotation(0.5);
            }
            if (gamepad1.dpad_left) {
                settherotation(0);
            }
            if (gamepad1.dpad_down) {
                flickys.setPosition(flickup);
            }
            if (gamepad1.dpad_up) {
                flickys.setPosition(flickdown);
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
            telemetry.addData("dist:", distancev);
            telemetry.addData("red:", redv);
            telemetry.addData("green:", greenv);
            telemetry.addData("blue:", bluev);
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