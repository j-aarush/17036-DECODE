package org.firstinspires.ftc.teamcode.DECODE.subsystems;

import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spina;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinb;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinc;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spind;
import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spino;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.bindings.BindingManager;


@TeleOp
public class servospindexer extends LinearOpMode {

    public static Servo leftspindex, rightspindex;
    NormalizedColorSensor colorSensor;
    View relativeLayout;
    Servo flickys;


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


//        leftspindex.setDirection(Servo.Direction.REVERSE);


        waitForStart();

        while(opModeIsActive()) {


//            leftinake.setPower(1);
//up is counterclockwise



            if (gamepad1.a) {
                settherotation(spina);  //SHOOTER 3 hoewfjepwa //0.45 1
            }
            if (gamepad1.b) {
                settherotation(spinb);
            }
            if (gamepad1.x) {
                settherotation(spinc); //SHOOTER 2

            }
            if (gamepad1.y) {
                settherotation(0); //SHOOTER 1  .hfgeiuawfheawpi  FITURE OUT SHOOTER 3
            }

//            if (gamepad1.left_bumper) {
//                settherotation(0.50);  //inatke pos 3
//            }
//            if (gamepad1.right_bumper) {
//                settherotation(0.28); //shooter pos 2
//            }
                if (gamepad1.dpad_down) {
                    settherotation(spind); //intake?
                }

//            gamepad1a.whenBecomesTrue(() -> settherotation(0)); //rough near zone
//
//            gamepad1b.whenBecomesTrue(() -> settherotation(0.1)); //rough far zone
//
//            gamepad1x.whenBecomesTrue(() -> settherotation(0)); //power save





        }
    }
}