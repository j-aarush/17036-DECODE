package org.firstinspires.ftc.teamcode.DECODE.tests;


import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DECODE.PIDs.CRaxonSpindexer;

import dev.nextftc.bindings.Button;


@TeleOp(name = "flicky")
public class flickservozero extends LinearOpMode {
    //Custom variable declaration for button speed change

    public static ElapsedTime intakeeee = new ElapsedTime(0);

    int flickswitch = -1;
    public static double flickup = 0.172, flickdown = 0.491;


    boolean truefalse;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        Servo flick, flickright;
        double speed;
        DcMotorEx leftintake;
        Servo left, right;
        intakeeee.reset();

        //Rintake; Lintake

        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        flick = hardwareMap.get(Servo.class, "flicky");
        flickright = hardwareMap.get(Servo.class, "flickyr");
        left = hardwareMap.get(Servo.class, "leftspindex");
        right = hardwareMap.get(Servo.class, "rightspindex");
        Servo rightwall = hardwareMap.get(Servo.class, "rightwall");
        Servo leftwall = hardwareMap.get(Servo.class, "leftwall");
        leftwall.setDirection(Servo.Direction.FORWARD);
        flickright = hardwareMap.get(Servo.class, "flickyr");
        flick.setDirection(Servo.Direction.REVERSE);
        flickright.setDirection(Servo.Direction.REVERSE);



//        leftintake = hardwareMap.get(DcMotorEx.class, "leftin");
//        leftintake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);




//        leftintake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); x


        byte speedx = 2;
        waitForStart();
        //Set Arm position
        if (isStopRequested()) return;
        //arm cannont move from here


        while (opModeIsActive()) {

            if (gamepad1.a) { //downnnnnn.
                flick.setPosition(flickdown);
                flickright.setPosition(flickdown);
            }
            if (gamepad1.b) { //down
                flick.setPosition(0.1515);
            }

            if (gamepad1.y) { //upppppp
                flick.setPosition(flickup);
                flickright.setPosition(flickup);
            }

            if (gamepad2.a) {
                rightwall.setPosition(0.45); //down
            }
            if (gamepad2.b) {
                rightwall.setPosition(0.285); //up
            }
            if (gamepad2.x) {
                rightwall.setPosition(1);
            }
            if (gamepad2.a) {
                leftwall.setPosition(0.259); //down
            }
            if (gamepad2.b) {
                leftwall.setPosition(0.42); //up
            }
            if (gamepad2.x) {
                leftwall.setPosition(1);
            }


//            switch (flickswitch) {
//                case 0:
//                    intakeeee.reset();
//                    flick.setPosition(0);
//                    if (intakeeee.time() > 0.25) {
//                        flickswitch = 1;}
//                    break;
//                case 1:
//                    flick.setPosition(0.5);
//                    break;
//            }


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


