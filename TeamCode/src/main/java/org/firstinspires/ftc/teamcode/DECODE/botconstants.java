package org.firstinspires.ftc.teamcode.DECODE;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;

public  class botconstants {

 public  static double spina = 0.243;
 public  static double spinb = 0.50;
    public static double spinc = 0.753;
  public static double flickup = 0.0, flickdown = 0.53;


  public static double kP = 0.1167, kV = 0.000434;
    PIDFController controller = new PIDFController(new PIDFCoefficients(0.1,0,0.006,0.000004));
//    targetV = 2447 + -51.2*diagonaldist + 0.753*diagonaldist*diagonaldist + -0.00437*diagonaldist*diagonaldist*diagonaldist + 0.0000091*diagonaldist*diagonaldist*diagonaldist*diagonaldist;

}
