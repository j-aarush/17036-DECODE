package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.control.KineticState;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class flywheelpid extends NextFTCOpMode {
    public flywheelpid() {
        addComponents(

        );
    }

    new RunToState(
            controlSystem, flywheel

)
    private final MotorEx flywheel = new MotorEx("flywheel");

    @Override public void onInit() { }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {

    }
    @Override public void onUpdate() {

    }
    @Override public void onStop() { }

}