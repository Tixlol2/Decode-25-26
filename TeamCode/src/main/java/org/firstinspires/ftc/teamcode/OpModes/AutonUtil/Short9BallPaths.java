package org.firstinspires.ftc.teamcode.OpModes.AutonUtil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

public class Short9BallPaths {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;

    public Short9BallPaths(Follower follower, UniConstants.teamColor color){
        if(color == UniConstants.teamColor.BLUE){
            blueShort(follower);
        } else {
            redShort(follower);
        }
    }

    public void redShort(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(122.000, 124.000), new Pose(92.000, 100.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(37))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.000, 100.000), new Pose(92.000, 83.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.000, 83.500), new Pose(116, 83.500))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.000, 100.000), new Pose(92.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.000, 60.000), new Pose(116, 60.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.000, 100.000), new Pose(90.000, 125.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void blueShort(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.blueGoalTopStartFacing, Poses.blueShortScore)
                )
                .setConstantHeadingInterpolation(Poses.blueShortScore.getHeading())
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.blueShortScore, Poses.readyBlueActiveTop)
                )
                .setLinearHeadingInterpolation(Poses.blueShortScore.getHeading(), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.readyBlueActiveTop, Poses.blueActiveTopStop)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.blueShortScore, Poses.readyBlueActiveMid)
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.readyBlueActiveMid, Poses.blueActiveMidStop)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(Poses.blueShortScore, Poses.blueParkAuto)
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}
