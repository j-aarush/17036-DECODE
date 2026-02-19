package org.firstinspires.ftc.teamcode.DECODE;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;

public  class botconstants {


 public static double autoendx, autoendy, autoendhead;
 public static Pose autoendpose;
 public static double autoendheading;
//    public  static double spina = 0.1355;
//
// public  static double spinb = 0.3855;
//    public static double spinc = 0.635;



    public static double spino = 0.03;
    public  static double spina = 0.285;

    public  static double spinb = 0.535;
    public static double spinc = 0.788;
    public static double spind = 1;

    public static double flickup = 0, flickdown = 0.30; //0.06

    public Servo flick, flickright, leftwall, rightwall;
    //MAKE SURE BOTH ARE REVERSED, set up in configs

  public static double kP = 0.1167, kV = 0.000434;
    PIDFController controller = new PIDFController(new PIDFCoefficients(0.1,0,0.006,0.000004));
//    targetV = 2447 + -51.2*diagonaldist + 0.753*diagonaldist*diagonaldist + -0.00437*diagonaldist*diagonaldist*diagonaldist + 0.0000091*diagonaldist*diagonaldist*diagonaldist*diagonaldist;



    public static double leftdown = 0.259;
    public static double rightdown = 0.47;
    public static double leftup = 0.42;
    public static double rightup = 0.285;

    public void setleftdown() {
        leftwall.setPosition(leftdown);
        rightwall.setPosition(rightup);
    }
    public void setrightdown() {
        leftwall.setPosition(leftup);
        rightwall.setPosition(rightdown);
    }
    public void bothwaldown() {
        leftwall.setPosition(leftdown);
        rightwall.setPosition(rightdown);
    }
    public void bothwallup() {
        leftwall.setPosition(leftup);
        rightwall.setPosition(rightup);
    }




}


//todo's
//limelight for heading/velocity
//cr spindexer pid
//retune pedro
//retune heading pid
//account for bot velocity for heading/velocity
//try field centric