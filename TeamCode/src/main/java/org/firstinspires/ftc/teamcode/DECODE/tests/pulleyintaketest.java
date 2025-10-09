package org.firstinspires.ftc.teamcode.DECODE.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous
public class pulleyintaketest extends LinearOpMode {
    //Custom variable declaration for button speed change



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx rightintake;
        DcMotorEx leftintake;
        double speed;


        //

        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        rightintake = hardwareMap.get(DcMotorEx.class, "rightin");
        leftintake = hardwareMap.get(DcMotorEx.class, "leftin");

        leftintake.setDirection(DcMotorEx.Direction.REVERSE);



        rightintake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftintake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftintake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        leftintake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); x


        byte speedx = 2;
        waitForStart();
        //Set Arm position
        if (isStopRequested()) return;
        //arm cannont move from here



        while (opModeIsActive()) {

           leftintake.setPower(gamepad1.right_stick_x);
            rightintake.setPower(gamepad1.right_stick_x);


            speed = leftintake.getVelocity();
            double velocity = speed * 60 / 28;


            telemetry.addData("not speed", velocity);
            telemetry.update();








        }
    }
}


