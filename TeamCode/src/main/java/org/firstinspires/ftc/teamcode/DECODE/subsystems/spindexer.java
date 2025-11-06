package org.firstinspires.ftc.teamcode.DECODE.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.DECODE.PIDs.RTPAxon;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.bindings.Button;
import dev.nextftc.control.KineticState;
import dev.nextftc.ftc.NextFTCOpMode;
import static org.firstinspires.ftc.teamcode.DECODE.PIDs.flywheelpid.*;
import static dev.nextftc.bindings.Bindings.button;


@TeleOp
public class spindexer extends LinearOpMode {

    CRServo leftspindex, rightspindex;
    RTPAxon leftleft, rightright;
    DcMotorEx angle;
    double spindexangle;

    @Override
    public void runOpMode() {


        angle = hardwareMap.get(DcMotorEx.class, "Lintake");
        angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftspindex = hardwareMap.get(CRServo.class, "leftspindex");
        AnalogInput leftencoder = hardwareMap.get(AnalogInput.class, "lefte");
        leftleft = new RTPAxon(leftspindex, angle);
        rightspindex = hardwareMap.get(CRServo.class, "rightspindex");
        AnalogInput rightencoder = hardwareMap.get(AnalogInput.class, "righte");
        rightright = new RTPAxon(rightspindex, angle);

        spindexangle = angle.getCurrentPosition() * 360 / 8192;


        while(opModeIsActive()) {

        leftleft.update();
        rightright.update();

        if (gamepad1.a) { leftleft.setTargetRotation(-150);
            rightright.setTargetRotation(-150); }
            if (gamepad1.b) { leftleft.setTargetRotation(150);
                rightright.setTargetRotation(150); }
            if (gamepad1.x) { leftleft.setTargetRotation(0);
                rightright.setTargetRotation(0); }

        telemetry.addData("encoder - ", spindexangle);
        telemetry.update();




    }
    }
}