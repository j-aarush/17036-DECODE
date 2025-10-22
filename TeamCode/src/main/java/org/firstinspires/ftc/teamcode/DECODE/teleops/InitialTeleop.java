package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.control.KineticState;
import dev.nextftc.ftc.NextFTCOpMode;
import static org.firstinspires.ftc.teamcode.DECODE.PIDs.flywheelpid.*;

@TeleOp(name = "teleop")
@Configurable
public class InitialTeleop extends NextFTCOpMode {

    boolean flywheeloff = false;
    boolean xpressed = false;

    @Override
    public void runOpMode() {


        waitForStart();

        while(opModeIsActive()) {

            boolean isXPressed = gamepad1.x;
            if (isXPressed && !xpressed) {
                flywheeloff = !flywheeloff;
            }
            xpressed = isXPressed;

            if (flywheeloff) {
                flywheel.setPower(0.2467);
            } else {
                flywheelvelocity = flywheel.getVelocity();
                KineticState currentState = new KineticState(0, flywheelvelocity, 0.0); //figure out velocity (is it in ticks?!?)
                velocityControlWithFeedforwardExample(currentState);
            }

            if (gamepad1.a) {
                configvelocity = 1300;
            }
            if (gamepad1.b) {
                configvelocity = 1520;
            }

            telemetry.addData("output real velocity:", flywheel.getVelocity());
            telemetry.addData("input velocity:", configvelocity);
            telemetry.addData("Flywheel PID", flywheeloff ? "Disabled (0.2 power)" : "Enabled");
            telemetry.update();

//claire skibidi toilet ohio

        }

    }
}
