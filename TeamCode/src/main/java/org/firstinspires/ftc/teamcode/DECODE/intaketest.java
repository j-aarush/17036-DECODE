package org.firstinspires.ftc.teamcode.DECODE;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous
public class intaketest extends LinearOpMode {
    //Custom variable declaration for button speed change



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor rightintake;
        DcMotor leftintake;


        //

        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        rightintake = hardwareMap.get(DcMotor.class, "rightin");
        leftintake = hardwareMap.get(DcMotor.class, "leftin");

        leftintake.setDirection(DcMotor.Direction.REVERSE);



        rightintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        byte speedx = 2;
        waitForStart();
        //Set Arm position
        if (isStopRequested()) return;
        //arm cannont move from here



        while (opModeIsActive()) {

           leftintake.setPower(gamepad1.right_stick_x);
            rightintake.setPower(gamepad1.right_stick_x);








        }
    }
}


