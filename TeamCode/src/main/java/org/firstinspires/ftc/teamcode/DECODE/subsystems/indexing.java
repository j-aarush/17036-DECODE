//package org.firstinspires.ftc.teamcode.DECODE.subsystems;
//
//import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spina;
//import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinb;
//import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinc;
//import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinflickdown;
//import static org.firstinspires.ftc.teamcode.DECODE.botconstants.spinflickup;
//
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class indexing {
//    public static Servo leftspindex, rightspindex, flickys, flickright;
//    int PPG_PP;
//    public static ElapsedTime intakeeee = new ElapsedTime(0);
//
//    public static ElapsedTime getIntakeeee() {
//        return intakeeee;
//    }
//
//    public void PPG_AND_PP() {
//        switch (PPG_PP) {
//            case 5:
//                if (intakeeee.time() > 0.0005) {
//                    headingLock = true;
//                    intaekstage = 6;
//                    intakeeee.reset();}
//                break;
//            case 6:
//                settherotation(spina);
//                if (!follower.isBusy()) {
//                    intaekstage = 7;
//                    intakeeee.reset();}
//                break;
//            case 7:
//                if (holdshooting) {
//                    parksettherotation(0.1);
//                }
//                spinflickup();
//                if (intakeeee.time() > 0.05) {
//                    intaekstage = 8;
//                    intakeeee.reset();}
//                break;
//            case 8:
//                spinflickdown();
//                if (intakeeee.time() > 0.05) {
//                    intaekstage = 9;
//                    intakeeee.reset();}
//                break;
//            case 9:
//                settherotation(spinb);
//                if (intakeeee.time() > 0.56) {
//                    intaekstage = 10;
//                    intakeeee.reset();}
//                break;
//            case 10:
//                spinflickup();
//                if (intakeeee.time() > 0.05) {
//                    intaekstage = 11;
//                    intakeeee.reset();}
//                break;
//            case 11:
//                spinflickdown();
//                if (intakeeee.time() > 0.05) {
//                    intaekstage = 12;
//                    intakeeee.reset();}
//                break;
//            case 12:
//                settherotation(spinc);
//                if (intakeeee.time() > 0.57) {
//                    intaekstage = 13;
//                    intakeeee.reset();}
//                break;
//            case 13:
//                spinflickup();
//                if (intakeeee.time() > 0.05) {
//                    headingLock = false;
//                    intaekstage = 14;
//                    intakeeee.reset();}
//                break;
//            case 14:
//                parksettherotation(0);
//                headingLock = false;
//                spinflickdown();
//                if (intakeeee.time() > 0.05) {
//                    intaekstage = -1;
//                    intakeeee.reset();
//                    settherotation(spina);}
//                break;
//        }
//    }
//
//}
