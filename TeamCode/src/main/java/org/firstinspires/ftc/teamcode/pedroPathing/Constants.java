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
            .mass(12.8)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .forwardZeroPowerAcceleration(-44)
            .lateralZeroPowerAcceleration(-69.9)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.068, 0, 0.0005, 0.117))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.095, 0, 0.0005, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.0009, 0.077))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0038, 0, 0.0005, 0.6, 0.081))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.003, 0, 0, 0.6, 0))
            .centripetalScaling(0)
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
            .xVelocity(78.7)
            .yVelocity(58.9)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.89)
            .strafePodX(2.56)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.38, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
