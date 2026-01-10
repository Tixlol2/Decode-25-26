package org.firstinspires.ftc.teamcode.OpModes.AutonUtil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

public class Short9BallPaths {

    public PathChain StartShoot;
    public PathChain ReadyIntakeTop;
    public PathChain IntakeTop;
    public PathChain ReadyIntakeMid;
    public PathChain IntakeMid;
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
                .build();

        Park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.blueShortScore, Poses.blueParkAuto)
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}
