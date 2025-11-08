package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
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




    Button gamepad1a = button(() -> gamepad1.a);
    Button gamepad1b = button(() -> gamepad1.b);
    Button gamepad1x = button(() -> gamepad1.x);

    DcMotorEx FL, FR, BL, BR, leftinake, rightinake;
    Servo flicky;



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



        waitForStart();
        colorSensor.setGain(12);

        while(opModeIsActive()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            greenv = colors.green;
            bluev = colors.blue;

            if (greenv > 0.02 && greenv < 0.04 && bluev > 0.02 && bluev < 0.039) {
                move = true;
                intakeeee.startTime();
            } else {
                move = false;
            }


            //get data from hub; store as variables at beginning of loop

            gamepad1a.whenBecomesTrue(() -> configvelocity = 1267); //rough near zone

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


            shooter();

//            if (gamepad2.a) {
//                BR.setPower(1);
//            } else if (gamepad2.b) {
//                BL.setPower(1);
//            }
//            else if (gamepad2.x) {
//                FR.setPower(1);
//            }
//            else if (gamepad2.y) {
//                FL.setPower(1);
//            } else if (gamepad2.left_bumper) {
//                BR.setPower(-1);
//                FR.setPower(-1);
//                FL.setPower(1);
//                BL.setPower(1);
//            } else{
//                BR.setPower(0);
//                FR.setPower(0);
//                FL.setPower(0);
//                BL.setPower(0);
//            }
            double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x - rx) / denominator;
            double frontRightPower = (y - x + rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

//

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