package org.firstinspires.ftc.teamcode.OpModes.Util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BluePaths{

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;


    public BluePaths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.000, 124.000), new Pose(45.000, 100.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(144))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.000, 100.000), new Pose(45.000, 83.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.000, 83.000), new Pose(20.000, 83.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.000, 83.000), new Pose(45.000, 100.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.000, 100.000), new Pose(45.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.000, 60.000), new Pose(20.000, 60.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.000, 60.000), new Pose(45.000, 100.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();
        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45, 10), new Pose(37, 135))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();
    }
}
