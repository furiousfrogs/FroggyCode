package org.firstinspires.ftc.teamcode.pedroPathing;
//FIXXXXXXXXXXXXXXXXXXXXXXXXX
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.9)
            .forwardZeroPowerAcceleration(-33.74)
            .lateralZeroPowerAcceleration(-66.01)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.0545, 0, 0.008, 0.08))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.001,0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.001, 0.0575))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.4,0,0.004,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.00423, 0, 0.000006, 0.6, 0.0675))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.000154, 0, 0.000008, 0.6, 0))
            //centripetal
            .centripetalScaling(0.0001)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(89.45)
            .yVelocity(68.23)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.25)
            .strafePodX(-3.75)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1.3);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
