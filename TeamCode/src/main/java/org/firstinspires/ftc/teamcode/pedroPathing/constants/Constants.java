package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(0)
            .forwardZeroPowerAcceleration(-39.62117650174046)
            .lateralZeroPowerAcceleration(-73.3436749528114)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.008, 0, 0.0002, 0, 0)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName(UniConstants.DRIVE_FRONT_LEFT_STRING)
            .leftRearMotorName(UniConstants.DRIVE_BACK_LEFT_STRING)
            .rightFrontMotorName(UniConstants.DRIVE_FRONT_RIGHT_STRING)
            .rightRearMotorName(UniConstants.DRIVE_BACK_RIGHT_STRING)
            .leftFrontMotorDirection(UniConstants.DRIVE_FRONT_LEFT_DIRECTION)
            .leftRearMotorDirection(UniConstants.DRIVE_BACK_LEFT_DIRECTION)
            .rightFrontMotorDirection(UniConstants.DRIVE_FRONT_RIGHT_DIRECTION)
            .rightRearMotorDirection(UniConstants.DRIVE_BACK_RIGHT_DIRECTION)
            .xVelocity(58.62645796861983)
            .yVelocity(43.764789808914855);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(UniConstants.PINPOINT_STRING)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
