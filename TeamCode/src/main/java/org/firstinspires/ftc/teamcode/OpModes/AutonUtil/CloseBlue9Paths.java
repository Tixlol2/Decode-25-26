package org.firstinspires.ftc.teamcode.OpModes.AutonUtil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class CloseBlue9Paths {
    public PathChain Path1;
public PathChain Path2;
public PathChain Path3;
public PathChain Path4;
public PathChain Path5;
public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;

    public CloseBlue9Paths(Follower follower) {
      Path1 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(32.000, 135.500),
            
            new Pose(56.000, 100.000)
          )
        ).setConstantHeadingInterpolation(Math.toRadians(90))
        
        .build();

Path2 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(56.000, 100.000),
            
            new Pose(56.000, 84.500)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
        
        .build();

Path3 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(56.000, 84.500),
            
            new Pose(14.000, 84.500)
          )
        ).setConstantHeadingInterpolation(Math.toRadians(180))
        
        .build();

Path4 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(14.000, 84.500),
            
            new Pose(56.000, 100.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
        
        .build();

Path5 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(56.000, 100.000),
            
            new Pose(56.000, 60.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
        
        .build();

Path6 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(56.000, 60.000),
            
            new Pose(14.000, 60.000)
          )
        ).setConstantHeadingInterpolation(Math.toRadians(180))
        
        .build();

Path7 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(14.000, 60.000),
            
            new Pose(56.000, 100.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
        
        .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 100.000),

                                new Pose(56.000, 120.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
  }
  