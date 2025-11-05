package org.firstinspires.ftc.teamcode.DECODE.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.DECODE.PIDs.RTPAxon;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.bindings.Button;
import dev.nextftc.control.KineticState;
import dev.nextftc.ftc.NextFTCOpMode;
import static org.firstinspires.ftc.teamcode.DECODE.PIDs.flywheelpid.*;
import static dev.nextftc.bindings.Bindings.button;


@TeleOp
public class spindexer extends OpMode {

    CRServo leftspindex, rightspindex;
    RTPAxon leftleft, rightright;

    @Override
    public void init() {


        leftspindex = hardwareMap.get(CRServo.class, "leftspindex");
        AnalogInput leftencoder = hardwareMap.get(AnalogInput.class, "lefte");
        leftleft = new RTPAxon(leftspindex, leftencoder);
        rightspindex = hardwareMap.get(CRServo.class, "rightspindex");
        AnalogInput rightencoder = hardwareMap.get(AnalogInput.class, "righte");
        rightright = new RTPAxon(rightspindex, rightencoder);


    }
    @Override
    public void loop() {

        leftleft.update();
        rightright.update();

        if (gamepad1.a) { leftleft.setTargetRotation(0);
            rightright.setTargetRotation(0); }
        if (gamepad1.b) { leftleft.setTargetRotation(120);
            rightright.setTargetRotation(120); }





    }
}