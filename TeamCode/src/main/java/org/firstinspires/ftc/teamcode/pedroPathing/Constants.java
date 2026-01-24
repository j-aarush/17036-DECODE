package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.8)
            .forwardZeroPowerAcceleration(-36.6021)
            .lateralZeroPowerAcceleration(-72.855)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
//            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07,0,0.0167,0.0267))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.015,0.002))
            .headingPIDFCoefficients(new PIDFCoefficients(0.67,0,0.03,0.145)) //0.85,  0.03   0.02
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3.4,0,0.015,0.025)) //1.5     /0.009
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.086,0.05,0,0.6, 0.00001)) //0.2, 0.00045
            .centripetalScaling(0.00025) //0.002
            .holdPointHeadingScaling(1)
//            .setCoefficientsHeadingPIDF(new PIDFCoefficients(0,0,0,0))
            ;
    public static PathConstraints pathConstraints = new PathConstraints(0.975, 50, 1, 1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(90.44085288)
            .yVelocity(70.95980667)
            .maxPower(1)

            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.57)
            .strafePodX(3.78)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
