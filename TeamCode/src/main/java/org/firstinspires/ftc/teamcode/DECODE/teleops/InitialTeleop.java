package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

    DcMotorEx FL, FR, BL, BR, leftinake, rightinake;

    @Override
    public void runOpMode() {

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        leftinake = hardwareMap.get(DcMotorEx.class, "Lintake");


        waitForStart();

        while(opModeIsActive()) {

            //get data from hub; store as variables at beginning of loop

            gamepad1a.whenBecomesTrue(() -> configvelocity = 1267); //rough near zone

            gamepad1b.whenBecomesTrue(() -> configvelocity = 1520); //rough far zone

            gamepad1x.whenBecomesTrue(() -> configvelocity = 350); //power save

            shooter();

//            if (gamepad2.a) {
//                BR.setPower(1);
//            } else if (gamepad2.b) {
//                BL.setPower(1);
//            }
//            else if (gamepad2.x) {
//                FR.setPower(1);
//            }
//            else if (gamepad2.y) {
//                FL.setPower(1);
//            } else if (gamepad2.left_bumper) {
//                BR.setPower(-1);
//                FR.setPower(-1);
//                FL.setPower(1);
//                BL.setPower(1);
                leftinake.setPower(1);
//            } else{
//                BR.setPower(0);
//                FR.setPower(0);
//                FL.setPower(0);
//                BL.setPower(0);
//            }
            double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x - rx) / denominator;
            double frontRightPower = (y - x + rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

//

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