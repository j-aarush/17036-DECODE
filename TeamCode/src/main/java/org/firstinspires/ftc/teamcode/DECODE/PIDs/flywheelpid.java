package org.firstinspires.ftc.teamcode.DECODE.PIDs;

import android.health.connect.datatypes.units.Power;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class flywheelpid extends NextFTCOpMode {
    public flywheelpid() {
        addComponents(

        );




    }


    double flywheelvelocity;


    public static MotorEx flywheel = new MotorEx("flywheel");
    public static ServoEx flickerrr = new ServoEx("flicky");


    public static void velocityControlWithFeedforwardExample(KineticState currentstate) {
        // Create a velocity controller with PID and feedforward
        ControlSystem controller = ControlSystem.builder()
                .velPid(0.1, 0.01, 0.05) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
//                .basicFF(0.02, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        // Set the goal velocity to 500 units per second
        controller.setGoal(new KineticState(0.0, 1300, 0.0));

        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity
        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity

        double power = controller.calculate(currentstate);
        flywheel.setPower(power);

        // Apply power to your motor
        System.out.println("Power to apply: " + power);
    }

    @Override public void onInit() { }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {

    }
    @Override public void onUpdate() {
        flywheelvelocity = flywheel.getVelocity();
        KineticState currentState = new KineticState(0, flywheelvelocity, 0.0); //figure out velocity (is it in ticks?!?)
         velocityControlWithFeedforwardExample(currentState);

        if (gamepad1.a) {
            flickerrr.setPosition(0.4);
        }
        if (gamepad1.b) {
            flickerrr.setPosition(0.0670);
        }



    }

}