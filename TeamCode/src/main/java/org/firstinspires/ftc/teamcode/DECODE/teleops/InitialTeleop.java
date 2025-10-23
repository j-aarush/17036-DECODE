package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.control.KineticState;
import dev.nextftc.ftc.NextFTCOpMode;
import static org.firstinspires.ftc.teamcode.DECODE.PIDs.flywheelpid.*;



@TeleOp(name = "teleop")
@Configurable
public class InitialTeleop extends NextFTCOpMode {


    @Override
    public void runOpMode() {


        waitForStart();

        while(opModeIsActive()) {

            //get data from hub; store as variables at beginning of loop
            flywheelvelocity = flywheel.getVelocity();


            //HOW TO DO THIS WITH NEXTFTC ASDL;FAHUEIJAORH8293PIEOW;FD SOBBBBBBBBBBBBBBBBBBBBB 😭
            if (gamepad1.a) {
                configvelocity = 1300;
            }
            if (gamepad1.b) {
                configvelocity = 1520;
            }
            if (gamepad1.x) {
                configvelocity = 350;
            }

            KineticState currentState = new KineticState(0, flywheelvelocity, 0.0); //figure out velocity (is it in ticks?!?)
            velocityControlWithFeedforwardExample(currentState);

            telemetry.addData("output real velocity:", flywheel.getVelocity());
            telemetry.addData("input velocity:", configvelocity);
            telemetry.update();

//claire skibidi toilet ohio

        }

    }
}
