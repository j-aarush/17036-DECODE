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
            .mass(11.7)
            .forwardZeroPowerAcceleration(-34.836)
            .lateralZeroPowerAcceleration(-68.1145)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
//            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07,0,0.0167,0.0267))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.015,0.002))
            .headingPIDFCoefficients(new PIDFCoefficients(1.767,0,0.0267,0.02))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.67,0,0.0167,0.0167))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.045,0.00045,0.008,0.6, 0.000001)) //0.03,0.00045,0.001,0.6, 0.00001
            .centripetalScaling(0.0002)
            ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.9, 1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(80.4155679)
            .yVelocity(57.408293)
            .maxPower(1)

            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
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
