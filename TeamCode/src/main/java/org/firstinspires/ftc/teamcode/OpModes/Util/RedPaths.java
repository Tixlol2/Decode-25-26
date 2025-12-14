package org.firstinspires.ftc.teamcode.OpModes.Util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedPaths {

  public PathChain Path1;
  public PathChain Path2;
  public PathChain Path3;
  public PathChain Path4;
  public PathChain Path5;
  public PathChain Path6;
  public PathChain Path7;
  public PathChain Path8;

  public RedPaths(Follower follower) {
    Path1 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(122.000, 125.000), new Pose(93.000, 105.000))
      )
      .setConstantHeadingInterpolation(Math.toRadians(36))
      .build();

    Path2 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(93.000, 105.000), new Pose(93.000, 83.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
      .build();

    Path3 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(93.000, 83.000), new Pose(120.000, 83.000))
      )
      .setConstantHeadingInterpolation(Math.toRadians(0))
      .build();

    Path4 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(120.000, 83.000), new Pose(93.000, 105.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
      .build();

    Path5 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(93.000, 105.000), new Pose(93.000, 60.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
      .build();

    Path6 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(93.000, 60.000), new Pose(125.000, 60.000))
      )
      .setConstantHeadingInterpolation(Math.toRadians(0))
      .build();

    Path7 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(125.000, 60.000), new Pose(93.000, 105.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
      .build();

    Path8 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(93.000, 105.000), new Pose(93.000, 133.000))
      )
      .setConstantHeadingInterpolation(Math.toRadians(36))
      .build();
  }
}
