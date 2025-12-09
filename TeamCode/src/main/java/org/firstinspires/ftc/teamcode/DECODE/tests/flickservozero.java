package org.firstinspires.ftc.teamcode.DECODE.tests;


import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DECODE.PIDs.CRaxonSpindexer;

import dev.nextftc.bindings.Button;


@Autonomous
public class flickservozero extends LinearOpMode {
    //Custom variable declaration for button speed change

    public static ElapsedTime intakeeee = new ElapsedTime(0);

    int flickswitch = -1;

    boolean truefalse;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        Servo flick;
        double speed;
        DcMotorEx leftintake;
        Servo left, right;
        intakeeee.reset();

        //Rintake; Lintake

        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        flick = hardwareMap.get(Servo.class, "flicky");
        left = hardwareMap.get(Servo.class, "leftspindex");
        right = hardwareMap.get(Servo.class, "rightspindex");
        flick.setDirection(Servo.Direction.FORWARD);



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
                flick.setPosition(0.0);
            }
            if (gamepad1.b) { //down
                flickswitch = 0;
            }


            switch (flickswitch) {
                case 0:
                    intakeeee.reset();
                    flick.setPosition(0);
                    if (intakeeee.time() > 0.25) {
                        flickswitch = 1;}
                    break;
                case 1:
                    flick.setPosition(0.5);
                    break;
            }


            if (gamepad2.a) {
                left.setPosition(0);
            }
            if (gamepad2.b) {
                right.setPosition(0);
            }


//            leftintake.setPower(0.7);

//            speed = leftintake.getVelocity();
//            double velocity = speed * 60 / 28;




            telemetry.update();








        }
    }
}


