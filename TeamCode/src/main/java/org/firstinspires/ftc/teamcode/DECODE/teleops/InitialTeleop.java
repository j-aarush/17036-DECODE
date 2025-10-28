package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.Button;
import dev.nextftc.control.KineticState;
import dev.nextftc.ftc.NextFTCOpMode;
import static org.firstinspires.ftc.teamcode.DECODE.PIDs.flywheelpid.*;
import static dev.nextftc.bindings.Bindings.button;


@TeleOp(name = "teleop")
@Configurable
public class InitialTeleop extends NextFTCOpMode {

    Button gamepad1a = button(() -> gamepad1.a);
    Button gamepad1b = button(() -> gamepad1.b);
    Button gamepad1x = button(() -> gamepad1.x);

    @Override
    public void runOpMode() {


        waitForStart();

        while(opModeIsActive()) {

            //get data from hub; store as variables at beginning of loop

            gamepad1a.whenBecomesTrue(() -> configvelocity = 1267); //rough near zone

            gamepad1b.whenBecomesTrue(() -> configvelocity = 1520); //rough far zone

            gamepad1x.whenBecomesTrue(() -> configvelocity = 350); //power save

            shooter();


            telemetry.addData("output real velocity:", flywheelvelocity);
            telemetry.addData("input velocity:", configvelocity);
            telemetry.update();

//claire skibidi toilet ohio

        }

    }
}
//coding todos:
//set up pinpoint/pedro
//format as much code as possible into seperate subsystem classes
//heading lock via ll/pp
//indexing
//formula for flywheel speed