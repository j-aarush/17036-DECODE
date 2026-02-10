package org.firstinspires.ftc.teamcode.DECODE;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;

public  class botconstants {


 public static double autoendx, autoendy, autoendhead;
 public static Pose autoendpose;
 public static double autoendheading;
//    public  static double spina = 0.1355;
//
// public  static double spinb = 0.3855;
//    public static double spinc = 0.635;

    public  static double spina = 0.257;

    public  static double spinb = 0.508;
    public static double spinc = 0.749;

    public static double flickup = 0.172, flickdown = 0.491;


  public static double kP = 0.1167, kV = 0.000434;
    PIDFController controller = new PIDFController(new PIDFCoefficients(0.1,0,0.006,0.000004));
//    targetV = 2447 + -51.2*diagonaldist + 0.753*diagonaldist*diagonaldist + -0.00437*diagonaldist*diagonaldist*diagonaldist + 0.0000091*diagonaldist*diagonaldist*diagonaldist*diagonaldist;

}


//todo's
//limelight for heading/velocity
//cr spindexer pid
//retune pedro
//retune heading pid
//account for bot velocity for heading/velocity
//try field centric