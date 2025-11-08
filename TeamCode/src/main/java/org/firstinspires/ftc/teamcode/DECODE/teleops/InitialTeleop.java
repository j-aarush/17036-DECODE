package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.bindings.Button;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.NextFTCOpMode;
import static org.firstinspires.ftc.teamcode.DECODE.PIDs.flywheelpid.*;
import static org.firstinspires.ftc.teamcode.DECODE.subsystems.intaketransfer.intakeeee;
import static dev.nextftc.bindings.Bindings.button;
import org.firstinspires.ftc.teamcode.DECODE.subsystems.intaketransfer;


@TeleOp(name = "teleop")
@Configurable
public class InitialTeleop extends NextFTCOpMode {
    public static  NormalizedColorSensor colorSensor;
    public static ElapsedTime intakeeee = new ElapsedTime();
    public static Servo leftspindex, rightspindex;
    public void settherotation(double rotationn) {
        leftspindex.setPosition(rotationn);
        rightspindex.setPosition(rotationn);
    }

    public static DcMotorEx leftintake;
    float greenv, bluev;
    boolean move;
    int counter = 0;
    int shootercounter = 0;
    double rotationpos;


    public Command intakestages = new LambdaCommand()
            .setStart(() -> {
                rotationpos = 0.175;
                settherotation(rotationpos); //first pos
                leftintake.setPower(1);
            })
            .setUpdate(() -> {
                if (move && intakeeee.time() > 500) {
                    rotationpos = rotationpos + 0.245;
                    settherotation(rotationpos);
                    intakeeee.reset();
                    counter++;
                }
            })
            .setStop(interrupted -> {
            })
            .setIsDone(() -> {
                if (counter == 2) {
                    counter = 0;
                    return true;
                }
                return false;
            }) // Returns if the command has finished
            .requires(/* subsystems the command implements */)
            .setInterruptible(true)
            .named("intakeeeeeee"); // sets the name of the command; optional


    public Command shooterstages = new LambdaCommand()
            .setStart(() -> {
                rotationpos = 0.175;
                settherotation(rotationpos); //first pos
            })
            .setUpdate(() -> {
                    rotationpos = rotationpos + 0.245;
                    settherotation(rotationpos);
                    shootercounter++;
            })
            .setStop(interrupted -> {
            })
            .setIsDone(() -> {
                if (counter == 2) {
                    counter = 0;
                    return true;
                }
                return false;
            }) // Returns if the command has finished
            .requires(/* subsystems the command implements */)
            .setInterruptible(true)
            .named("intakeeeeeee"); // sets the name of the command; optional




    Button gamepad1a = button(() -> gamepad1.a);
    Button gamepad1b = button(() -> gamepad1.b);
    Button gamepad1x = button(() -> gamepad1.x);

    DcMotorEx FL, FR, BL, BR, leftinake, rightinake;



    @Override
    public void runOpMode() {

            Command myCommand = new SequentialGroup(
                    intakestages,
                    shooterstages
            );



        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakecolor");
        leftspindex = hardwareMap.get(Servo.class, "leftspindex");
        rightspindex = hardwareMap.get(Servo.class, "rightspindex");
        leftintake = hardwareMap.get(DcMotorEx.class, "Lintake");



        waitForStart();
        colorSensor.setGain(12);

        while(opModeIsActive()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            greenv = colors.green;
            bluev = colors.blue;

            if (greenv > 0.02 && greenv < 0.04 && bluev > 0.02 && bluev < 0.039) {
                move = true;
                intakeeee.startTime();
            }


            //get data from hub; store as variables at beginning of loop

            gamepad1a.whenBecomesTrue(() -> configvelocity = 1267); //rough near zone

            gamepad1b.whenBecomesTrue(() -> configvelocity = 1520); //rough far zone

            gamepad1x.whenBecomesTrue(() -> configvelocity = 350); //power save


            CommandManager.INSTANCE.scheduleCommand(myCommand);



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