package org.firstinspires.ftc.teamcode.DECODE.tests;

import static org.firstinspires.ftc.teamcode.DECODE.PIDs.flywheelpid.configvelocity;
import static org.firstinspires.ftc.teamcode.DECODE.PIDs.flywheelpid.flywheelvelocity;

import android.graphics.Color;
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
    NormalizedColorSensor intakecs, leftcs;
    View relativeLayout;
    Servo flickys;
    float greenintake, blueintake, greenleft, blueleft, greenright, blueright;

    final float[] hsvValuesintake = new float[3];
    final float[] hsvValuesright = new float[3];
    final float[] hsvValuesleft = new float[3];

    DcMotorEx leftinake;
//    Button gamepad1a = button(() -> gamepad1.a);
//    Button gamepad1b = button(() -> gamepad1.b);
//    Button gamepad1x = button(() -> gamepad1.x);

    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    boolean leftisgreen, leftispurple, rightisgreen, rightispurple, noballleft, noballright;


    @Override
    public void runOpMode() {

        flickys = hardwareMap.get(Servo.class, "flicky");
        flickys.setDirection(Servo.Direction.FORWARD);

        leftspindex = hardwareMap.get(Servo.class, "leftspindex");
        rightspindex = hardwareMap.get(Servo.class, "rightspindex");
        leftinake = hardwareMap.get(DcMotorEx.class, "Lintake");
        intakecs = hardwareMap.get(NormalizedColorSensor.class, "intakecolor");
        leftcs = hardwareMap.get(NormalizedColorSensor.class, "leftcs");




//        leftspindex.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        intakecs.setGain(12);
        leftcs.setGain(12);


        while(opModeIsActive()) {
            NormalizedRGBA colorintake = intakecs.getNormalizedColors();
            NormalizedRGBA colorleft = leftcs.getNormalizedColors();

            greenintake = colorintake.green;
            blueintake= colorintake.blue;


            greenleft = colorleft.green;
            blueleft = colorleft.blue;


            Color.colorToHSV(colorintake.toColor(), hsvValuesintake);
            Color.colorToHSV(colorleft.toColor(), hsvValuesleft);



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

            if (hsvValuesintake[0] > 130 && hsvValuesintake[0] < 176 && hsvValuesintake[2] > 0.019 && hsvValuesintake[2] < 0.041) {
                rightisgreen = true;
                noballright = false;
                rightispurple = false;
            } else if (hsvValuesintake[0] > 179 && hsvValuesintake[0] < 230 && hsvValuesintake[2] > 0.015 && hsvValuesintake[2] < 0.033) {
                rightisgreen = false;
                noballright = false;
                rightispurple = true;
            } else {
                rightispurple = false;
                rightisgreen = false;
                noballright = true;
            }

            if (hsvValuesleft[0] > 130 && hsvValuesleft[0] < 176 && hsvValuesleft[2] > 0.019 && hsvValuesleft[2] < 0.041) {
                leftisgreen = true;
                noballleft = false;
                leftispurple = false;
            } else if (hsvValuesleft[0] > 179 && hsvValuesleft[0] < 230 && hsvValuesleft[2] > 0.015 && hsvValuesleft[2] < 0.033) {
                leftisgreen = false;
                noballleft = false;
                leftispurple = true;
            } else {
                leftisgreen = false;
                noballleft = true;
                leftispurple = false;
            }





            if (hsvValuesleft[0] > 130 && hsvValuesleft[0] < 176 && hsvValuesleft[2] > 0.019 && hsvValuesleft[2] < 0.041) {
                leftisgreen = true;
                noballleft = false;
            } else leftisgreen = false;


            if (hsvValuesleft[0] > 179 && hsvValuesleft[0] < 230 && hsvValuesleft[2] > 0.015 && hsvValuesleft[2] < 0.031) {
                leftispurple = true;
                noballleft = false;
            } else leftispurple = false;



            telemetry.addLine()
                    .addData("intakeGreen", "%.3f", greenintake)
                    .addData("intakeBlue", "%.3f", blueintake);
            telemetry.addLine()
                    .addData("leftGreen", "%.3f", greenleft)
                    .addData("leftBlue", "%.3f", blueleft);
            telemetry.addLine()
                    .addData("intakeHue", "%.3f", hsvValuesintake[0])
                    .addData("intakeSaturation", "%.3f", hsvValuesintake[2]);
            telemetry.addLine()
                    .addData("leftHue", "%.3f", hsvValuesleft[0])
                    .addData("leftSaturation", "%.3f", hsvValuesleft[2]);
            telemetry.addData("leftisgreen", leftisgreen);
            telemetry.addData("leftispurple", leftispurple);
            telemetry.addData("leftisnone", noballleft);
            telemetry.addData("rightisgreen", rightisgreen);
            telemetry.addData("rightispurple", rightispurple);
            telemetry.addData("rightisnone", noballright);



            //green hue: 130-176
            //green saturation: 0.019 - 0.041
            //purple hue: 179-230
            //purple saturation 0.015-0.03

            telemetry.update();

//sidegreen values:


        }
    }
}