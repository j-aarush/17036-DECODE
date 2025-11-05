package org.firstinspires.ftc.teamcode.DECODE.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous
public class flickservozero extends LinearOpMode {
    //Custom variable declaration for button speed change



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        Servo flick;
        double speed;
        DcMotorEx leftintake;



        //Rintake; Lintake

        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        flick = hardwareMap.get(Servo.class, "flicky");
//        leftintake = hardwareMap.get(DcMotorEx.class, "leftin");
//        leftintake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);




//        leftintake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); x


        byte speedx = 2;
        waitForStart();
        //Set Arm position
        if (isStopRequested()) return;
        //arm cannont move from here



        while (opModeIsActive()) {

            if (gamepad1.a) { //up.
                flick.setPosition(0);
            }
            if (gamepad1.b) { //down
                flick.setPosition(0.4);
            }

//            leftintake.setPower(0.7);

//            speed = leftintake.getVelocity();
//            double velocity = speed * 60 / 28;




            telemetry.update();








        }
    }
}


