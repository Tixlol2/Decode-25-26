package org.firstinspires.ftc.teamcode.OpModes.AutonUtil;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class CloseBlue9Paths {
    public PathChain ScanAndShoot;
    public PathChain MoveToActive1;
    public PathChain Active1;
    public PathChain Shoot2;
    public PathChain MovetoActive2;
    public PathChain Active2;
    public PathChain Shoot3;
    public PathChain Park;

    public CloseBlue9Paths(Follower follower) {
        ScanAndShoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(32.000, 135.500),

                                new Pose(56.000, 100.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))

                .build();

        MoveToActive1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 100.000),

                                new Pose(56.000, 84.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Active1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 84.500),

                                new Pose(14.000, 84.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Shoot2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.000, 84.500),

                                new Pose(56.000, 100.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                .build();

        MovetoActive2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 100.000),

                                new Pose(56.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                .build();

        Active2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 60.000),

                                new Pose(14.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Shoot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(14.000, 60.000),
                                new Pose(60.178, 67.205),
                                new Pose(56.000, 100.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 100.000),

                                new Pose(56.000, 120.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}
  