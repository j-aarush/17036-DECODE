package org.firstinspires.ftc.teamcode.DECODE.PIDs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class ffflywheel extends LinearOpMode {

    DcMotorEx flywheel;

    public static float targetV = 0;

    double kP = 0.1, kV = 0.00043;
    double error;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        waitForStart();

        while(opModeIsActive()) {
            error = targetV - flywheel.getVelocity();

            flywheel.setPower(kP * error + kV * targetV);

            if (gamepad1.a) {
                targetV = 1000;
            }
            if (gamepad1.b) {
                targetV = 1500;
            }

            telemetry.addData("targetV", targetV);
            telemetry.addData("velocity", flywheel.getVelocity());
            telemetry.update();

        }
    }
}
