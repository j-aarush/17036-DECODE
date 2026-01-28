package org.firstinspires.ftc.teamcode.DECODE.tests;

import static org.firstinspires.ftc.teamcode.DECODE.PIDs.flywheelpid.configvelocity;
import static org.firstinspires.ftc.teamcode.DECODE.PIDs.flywheelpid.flywheelvelocity;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.bindings.BindingManager;


@TeleOp
public class colorsensor extends LinearOpMode {

    public static Servo leftspindex, rightspindex;
    NormalizedColorSensor intakecs, rightcs, leftcs;
    View relativeLayout;
    Servo flickys;
    float greenintake, blueintake, greenleft, blueleft, greenright, blueright;


    DcMotorEx leftinake;
//    Button gamepad1a = button(() -> gamepad1.a);
//    Button gamepad1b = button(() -> gamepad1.b);
//    Button gamepad1x = button(() -> gamepad1.x);

    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }


    @Override
    public void runOpMode() {

        flickys = hardwareMap.get(Servo.class, "flicky");
        flickys.setDirection(Servo.Direction.FORWARD);

        leftspindex = hardwareMap.get(Servo.class, "leftspindex");
        rightspindex = hardwareMap.get(Servo.class, "rightspindex");
        leftinake = hardwareMap.get(DcMotorEx.class, "Lintake");
        intakecs = hardwareMap.get(NormalizedColorSensor.class, "intakecolor");
        rightcs = hardwareMap.get(NormalizedColorSensor.class, "rightcs");
        leftcs = hardwareMap.get(NormalizedColorSensor.class, "leftcs");




//        leftspindex.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        intakecs.setGain(100);
        rightcs.setGain(100);
        leftcs.setGain(100);


        while(opModeIsActive()) {
            NormalizedRGBA colorintake = intakecs.getNormalizedColors();
            NormalizedRGBA colorright = rightcs.getNormalizedColors();
            NormalizedRGBA colorleft = leftcs.getNormalizedColors();

            greenintake = colorintake.green;
            blueintake= colorintake.blue;

            greenright = colorright.green;
            blueright = colorright.blue;

            greenleft = colorleft.green;
            blueleft = colorleft.blue;




//            leftinake.setPower(1);
//up is counterclockwise


            if (gamepad1.a) {
                settherotation(0.495);  //SHOOTER 3 hoewfjepwa //0.45 1
            }
            if (gamepad1.b) {
                settherotation(0.75);
            }
            if (gamepad1.x) {
                settherotation(1); //SHOOTER 2

            }
            if (gamepad1.y) {
                settherotation(0.24); //SHOOTER 1  .hfgeiuawfheawpi  FITURE OUT SHOOTER 3
            }


            telemetry.addLine()
                    .addData("intakeGreen", "%.3f", greenintake)
                    .addData("intakeBlue", "%.3f", blueintake);
            telemetry.addLine()
                    .addData("leftGreen", "%.3f", greenleft)
                    .addData("leftBlue", "%.3f", blueleft);
            telemetry.addLine()
                    .addData("rightGreen", "%.3f", greenright)
                    .addData("rightBlue", "%.3f", blueright);



            //if green for back: 1.1 < green < 0.9;

            telemetry.update();

//sidegreen values:


        }
    }
}