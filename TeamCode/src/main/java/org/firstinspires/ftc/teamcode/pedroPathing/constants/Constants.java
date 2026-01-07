package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .forwardZeroPowerAcceleration(-44.27070972535246)
            .lateralZeroPowerAcceleration(-66.75944463360159)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0003)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(.67, 0, 0, 0.0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.005, 0, 0.000, 0.6, 0)
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
            .xVelocity(61.17509496493602)
            .yVelocity(50.46043047567052);



    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.5)
            .strafePodX(4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(UniConstants.PINPOINT_STRING)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            1,
            1.25
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
