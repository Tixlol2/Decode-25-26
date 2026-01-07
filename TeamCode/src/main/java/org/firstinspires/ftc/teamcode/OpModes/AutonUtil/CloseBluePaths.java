package org.firstinspires.ftc.teamcode.OpModes.AutonUtil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class CloseBluePaths {

  public PathChain Path1;
  public PathChain Path2;
  public PathChain Path3;
  public PathChain Path4;
  public PathChain Path5;
  public PathChain Path6;
  public PathChain Path7;
  public PathChain Path8;
  public PathChain Path9;
  public PathChain Path10;
  public PathChain Path11;
  public PathChain Path12;
  public PathChain Path13;

  public CloseBluePaths(Follower follower) {
    Path1 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(22.000, 124.000), new Pose(44, 112))
      )
      .setConstantHeadingInterpolation(Math.toRadians(144))
      .build();

    Path2 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(38, 116), new Pose(48.000, 56))
      )
      .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
      .build();

    Path3 = follower
            .pathBuilder()
            .addPath(
                    new BezierLine(new Pose(48.000, 56), new Pose(12, 62))
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

    Path4 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          new Pose(12, 64),
          new Pose(62.000, 62.000),
          new Pose(48.000, 96)
        )
      )
      .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
      .build();

    Path5 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(38, 116), new Pose(48.000, 83.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
      .build();

    Path6 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(48.000, 83.000), new Pose(10, 83.000))
      )
      .setConstantHeadingInterpolation(Math.toRadians(180))
      .build();

    Path7 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(12, 83.000), new Pose(44, 112))
      )
      .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
      .build();

    Path8 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(38, 116), new Pose(48.000, 35.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
      .build();

    Path9 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(48.000, 35.000), new Pose(8, 35.000))
      )
      .setConstantHeadingInterpolation(Math.toRadians(180))
      .build();

    Path10 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(12, 35.000), new Pose(44, 112))
      )
      .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
      .build();

    Path11 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(38, 116), new Pose(40.000, 122.000))
      )
      .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(35))
      .build();

    Path12 = follower
            .pathBuilder()
            .addPath(
                    new BezierLine(new Pose(44.000, 112.000), new Pose(38.000, 116.000))
            )
            .setConstantHeadingInterpolation(Math.toRadians(144))
            .build();

    Path13 = follower
            .pathBuilder()
            .addPath(
                    new BezierLine(new Pose(48, 96), new Pose(38.000, 116.000))
            )
            .setConstantHeadingInterpolation(Math.toRadians(90))
            .build();

  }
}
