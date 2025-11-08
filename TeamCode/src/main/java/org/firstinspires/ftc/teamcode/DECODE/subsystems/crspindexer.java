package org.firstinspires.ftc.teamcode.DECODE.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.DECODE.PIDs.CRaxonSpindexer;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;

import static dev.nextftc.bindings.Bindings.button;


@TeleOp
public class crspindexer extends LinearOpMode {

    CRServo leftspindex, rightspindex;
    CRaxonSpindexer leftleft, rightright;
    DcMotorEx angle;
    double spindexangle;
    Button gamepad1a = button(() -> gamepad1.a);
    Button gamepad1b = button(() -> gamepad1.b);
    Button gamepad1x = button(() -> gamepad1.x);

    void settherotation(double rotationn) {
        leftleft.setTargetRotation(rotationn);
        rightright.setTargetRotation(rotationn);
    }


    @Override
    public void runOpMode() {


        angle = hardwareMap.get(DcMotorEx.class, "Lintake");
        angle.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        angle.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        leftspindex = hardwareMap.get(CRServo.class, "leftspindex");
        AnalogInput leftencoder = hardwareMap.get(AnalogInput.class, "lefte");
        leftleft = new CRaxonSpindexer(leftspindex, angle);
        rightspindex = hardwareMap.get(CRServo.class, "rightspindex");
        AnalogInput rightencoder = hardwareMap.get(AnalogInput.class, "righte");
        rightright = new CRaxonSpindexer(rightspindex, angle);

        spindexangle = angle.getCurrentPosition() * 360 / 8192;


        while(opModeIsActive()) {

        leftleft.update();
        rightright.update();
        BindingManager.update();


            gamepad1a.whenBecomesTrue(() -> settherotation(-150)); //rough near zone

            gamepad1b.whenBecomesTrue(() -> settherotation(150)); //rough far zone

            gamepad1x.whenBecomesTrue(() -> settherotation(0)); //power save

        telemetry.addData("encoder - ", spindexangle);
        telemetry.update();




    }
    }
}