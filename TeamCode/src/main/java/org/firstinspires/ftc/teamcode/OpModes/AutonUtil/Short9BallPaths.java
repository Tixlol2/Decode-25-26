package org.firstinspires.ftc.teamcode.OpModes.AutonUtil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

public class Short9BallPaths {

    public PathChain StartShoot;
    public PathChain ReadyIntakeTop;
    public PathChain IntakeTop;
    public PathChain TopShoot;

    public PathChain ReadyIntakeMid;
    public PathChain IntakeMid;
    public PathChain MidShoot;

    public PathChain Park;

    public Short9BallPaths(Follower follower, UniConstants.teamColor color){
        if(color == UniConstants.teamColor.BLUE){
            blueShort(follower);
        } else {
            redShort(follower);
        }
    }

    public void redShort(Follower follower) {
        StartShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.redGoalTopStartFacing, Poses.redShortScore)
                )
                .setConstantHeadingInterpolation(Math.toRadians(37))
                .build();

        ReadyIntakeTop = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.redShortScore, Poses.readyRedActiveTop)
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                .build();

        IntakeTop = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.readyRedActiveTop, Poses.redActiveTopStop)
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        ReadyIntakeMid = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.redShortScore, Poses.readyRedActiveMid)
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                .build();

        IntakeMid = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.readyRedActiveMid, Poses.redActiveMidStop)
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.redShortScore, Poses.redParkAuto)
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        MidShoot = follower
                .pathBuilder()
                .addPath(new BezierCurve(Poses.redActiveMidStop, Poses.redMidCP, Poses.redShortScore))
                .setConstantHeadingInterpolation(Math.toRadians(37))
                .build();

        TopShoot = follower
                .pathBuilder()
                .addPath(new BezierLine(Poses.redActiveTopStop,  Poses.redShortScore))
                .setConstantHeadingInterpolation(Math.toRadians(37))
                .build();
    }

    public void blueShort(Follower follower) {
        StartShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.blueGoalTopStartFacing, Poses.blueShortScore)
                )
                .setConstantHeadingInterpolation(Poses.blueShortScore.getHeading())
                .build();

        ReadyIntakeTop = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.blueShortScore, Poses.readyBlueActiveTop)
                )
                .setLinearHeadingInterpolation(Poses.blueShortScore.getHeading(), Math.toRadians(180))
                .build();

        IntakeTop = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.readyBlueActiveTop, Poses.blueActiveTopStop)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setNoDeceleration()
                .build();

        ReadyIntakeMid = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.blueShortScore, Poses.readyBlueActiveMid)
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        IntakeMid = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.readyBlueActiveMid, Poses.blueActiveMidStop)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setNoDeceleration()
                .build();

        Park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.blueShortScore, Poses.blueParkAuto)
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setNoDeceleration()
                .build();

        TopShoot = follower
                .pathBuilder()
                .addPath(new BezierLine(Poses.blueActiveTopStop, Poses.blueShortScore))
                .setConstantHeadingInterpolation(Math.toRadians(144))
                .build();

        MidShoot = follower
                .pathBuilder()
                .addPath(new BezierCurve(Poses.blueActiveMidStop,Poses.blueMidCP, Poses.blueShortScore))
                .setConstantHeadingInterpolation(Math.toRadians(144))
                .build();

    }
}
