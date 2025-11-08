package org.firstinspires.ftc.teamcode.DECODE.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.DECODE.subsystems.servospindexer;
import org.firstinspires.ftc.teamcode.DECODE.subsystems.servospindexer;
import dev.nextftc.core.commands.Command;

public class intaketransfer extends NextFTCOpMode {

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

    @Override public void onInit() {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakecolor");
        leftspindex = hardwareMap.get(Servo.class, "leftspindex");
        rightspindex = hardwareMap.get(Servo.class, "rightspindex");
        leftintake = hardwareMap.get(DcMotorEx.class, "Lintake");

        waitForStart();
        colorSensor.setGain(12);


    }

    @Override public void onUpdate() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        greenv = colors.green;
        bluev = colors.blue;

        if (greenv > 0.02 && greenv < 0.04 && bluev > 0.02 && bluev < 0.039) {
            move = true;
            intakeeee.startTime();
        }







    }
}
