package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.controllable.RunToState;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class flywheelpid extends NextFTCOpMode {
    public flywheelpid() {
        addComponents(

        );




    }





    private final MotorEx flywheel = new MotorEx("flywheel");


    public static void velocityControlWithFeedforwardExample(KineticState currentstate) {
        // Create a velocity controller with PID and feedforward
        ControlSystem controller = ControlSystem.builder()
                .velPid(0.1, 0.01, 0.05) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
//                .basicFF(0.02, 0.0, 0.01) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        // Set the goal velocity to 500 units per second
        controller.setGoal(new KineticState(0.0, 500.0, 0.0));

        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity
        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity

        double power = controller.calculate(currentstate);

        // Apply power to your motor
        System.out.println("Power to apply: " + power);
    }

    @Override public void onInit() { }
    @Override public void onWaitForStart() { }
    @Override public void onStartButtonPressed() {

    }
    @Override public void onUpdate() {
        KineticState currentState = new KineticState(0, 450.0, 0.0); //figure out velocity (is it in ticks?!?)
         velocityControlWithFeedforwardExample(currentState);



    }

}