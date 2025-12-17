package org.firstinspires.ftc.teamcode.DECODE.subsystems;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.bindings.BindingManager;


@TeleOp
public class park extends LinearOpMode {

    public static Servo leftpark, rightpark;
    NormalizedColorSensor colorSensor;


    DcMotorEx leftinake;
//    Button gamepad1a = button(() -> gamepad1.a);
//    Button gamepad1b = button(() -> gamepad1.b);
//    Button gamepad1x = button(() -> gamepad1.x);

    public void settherotation(double rotationn) {
        leftpark.setPosition(rotationn);
        rightpark.setPosition(rotationn);
    }


    @Override
    public void runOpMode() {


        leftpark = hardwareMap.get(Servo.class, "leftpark");
        rightpark = hardwareMap.get(Servo.class, "rightpark");
        leftpark.setDirection(Servo.Direction.REVERSE);
        rightpark.setDirection(Servo.Direction.REVERSE);




        waitForStart();

        while(opModeIsActive()) {


//            leftinake.setPower(1);
//up is counterclockwise



            if (gamepad1.a) {
                leftpark.setPosition(0.5);
            }
            if (gamepad1.b) {
                leftpark.setPosition(1);

            }
            if (gamepad1.x) {
                rightpark.setPosition(0.2);

            }
            if (gamepad1.y) {
                rightpark.setPosition(0.8);

            }
//            if (gamepad1.left_bumper) {
//                settherotation(0.50);  //inatke pos 3
//            }
//            if (gamepad1.right_bumper) {
//                settherotation(0.28); //shooter pos 2
//            }
            if (gamepad1.dpad_down) {
            }

            if(gamepad2.a)
            {
            }
//            gamepad1a.whenBecomesTrue(() -> settherotation(0)); //rough near zone
//
//            gamepad1b.whenBecomesTrue(() -> settherotation(0.1)); //rough far zone
//
//            gamepad1x.whenBecomesTrue(() -> settherotation(0)); //power save





        }
    }
}