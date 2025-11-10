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


@TeleOp(name = "newtelop")
@Configurable
public class newteleop extends NextFTCOpMode {
    public static  NormalizedColorSensor colorSensor;
    public static ElapsedTime intakeeee = new ElapsedTime();
    public static Servo leftspindex, rightspindex;
    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    public static DcMotorEx intake;
    float greenv, bluev;
    boolean move;
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
    Button shootingfsm = button(() -> gamepad2.right_bumper);


    DcMotorEx FL, FR, BL, BR, leftinake, rightinake;
    Servo flicky;

    double botHeading;


    @Override
    public void runOpMode() {



        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        flicky = hardwareMap.get(Servo.class, "flicky");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakecolor");
        leftspindex = hardwareMap.get(Servo.class, "leftspindex");
        rightspindex = hardwareMap.get(Servo.class, "rightspindex");
        intake = hardwareMap.get(DcMotorEx.class, "Lintake");

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor


        waitForStart();
        colorSensor.setGain(12);

        while(opModeIsActive()) {
            greenv = colors.green;
            bluev = colors.blue;
            pinpoint.update();
            botHeading = pinpoint.getHeading(AngleUnit.DEGREES);



            if (greenv > 0.02 && greenv < 0.04 && bluev > 0.02 && bluev < 0.039) {
                move = true;
                intakeeee.startTime();
            } else {
                move = false;
            }


            //get data from hub; store as variables at beginning of loop

            gamepad1y.whenBecomesTrue(() -> configvelocity = 1267); //rough near zone

            gamepad1b.whenBecomesTrue(() -> configvelocity = 1520); //rough far zone

            gamepad1x.whenBecomesTrue(() -> configvelocity = 350); //power save


            if (gamepad1.right_bumper) {
                intaekstage = 0;
            }
            if (gamepad1.left_bumper) {
                shooterstage = 0;
            }

            switch (intaekstage) {
                case 0:
                    intake.setPower(1);
//                    rotationpos = 0;
                    settherotation(0); //first pos
                    if (move && intakeeee.seconds() > 0.5) {
//                        rotationpos = rotationpos + 0.25;
                        settherotation(0.25);
                        move = false;
                        intakeeee.reset();
                        intaekstage = 1;
                    }
                    break;
                case 1:
                    intake.setPower(1);
                    settherotation(0.25); //first pos
                    if (move && intakeeee.seconds() > 0.5) {
//                        rotationpos = rotationpos + 0.25;
                        settherotation(0.5);
                        move = false;
                        intakeeee.reset();
                        intaekstage = 2;
                    }
                    break;
                case 2:
                    intake.setPower(1);
//                rotationpos = 0.175;
                    settherotation(0.5);
//                    settherotation(rotationpos); //first pos
                if (move && intakeeee.seconds() > 0.5) {
                    settherotation(0.6167);
                    move = false;
                    intakeeee.reset();
                    intake.setPower(0);
                    intaekstage = -1;
                }
                break;
            }

            if (gamepad1.dpad_left) {
                flicky.setPosition(0.27);
            }
            if (gamepad1.dpad_right) {
                flicky.setPosition(0.5);
            }



            switch (shooterstage) {
                case 0:
//                    rotationpos = 0.6167;
                    settherotation(0.6167); //first pos
                        settherotation(rotationpos);
//                        flickerrr.setPosition(0.067);
//                        flickerrr.setPosition(0.4);
                        intaekstage = 1;
                    break;
                case 1:
                    settherotation(rotationpos); //first pos
                        rotationpos = rotationpos - 0.253;
                        settherotation(rotationpos);
//                    flickerrr.setPosition(0.067);
//                    flickerrr.setPosition(0.4);

                    intaekstage = 2;

                    break;
                case 2:
                    rotationpos = 0.175;
                    settherotation(rotationpos); //first pos
                        rotationpos = rotationpos - 0.253;
                        settherotation(rotationpos);
//                    flickerrr.setPosition(0.067);
//                    flickerrr.setPosition(0.4);
                    intaekstage = 0;
                        shooterstage = -1;
                    break;
            }




            double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x;
            double rx = gamepad2.right_stick_x;

            if (gamepad1.options) {
                pinpoint.setHeading(0, AngleUnit.DEGREES);
            }

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            shooter();


            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

            telemetry.addData("output real velocity:", flywheelvelocity);
            telemetry.addData("input velocity:", configvelocity);
            telemetry.update();
//claire skibidi toilet ohio

        }

    }

}
//coding todos:
//set up pinpoint/pedro
//format as much code as possible into seperate subsystem classes
//heading lock via ll/pp
//indexing
//formula for flywheel speed


//driver: field centric joysticks, shooter mode on/off (HOLD to keep shooting, not just press)
//opperator: *heading lock on/off*, shooter speed, intake mode on/off,
//make single driver code for funsies (later)
//MONDAY -- FINISH TELEOP BY ADDING HEADING LOCK, MAKING SHOOTING A HOLD BUTTON ( NOT PRESS BUTTON), TUNE PEDRO