package org.firstinspires.ftc.teamcode.OpModes;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSortingSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

//Written by Noah Nottingham - 6566 Circuit Breakers

@Autonomous(name = "9 Far Ball Pathing", group = "Main") //The name and group
@Configurable
public class Far9Ball extends NextFTCOpMode {
    public static Pose endPose = new Pose();
    JoinedTelemetry joinedTelemetry;

    //    public static Pose startPose = Poses.blueGoalTopStartFacing;
    Far9Paths paths;

    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        ); //Subsystems
    }

    @Override
    public void onInit() {
        Robot.inTeleop = false;
        joinedTelemetry = Robot.INSTANCE.getJoinedTelemetry();
        TurretSubsystem.INSTANCE.init();
    }


    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            Robot.color = Robot.teamColor.RED;
        } else if (gamepad1.b) {
            Robot.color = Robot.teamColor.BLUE;
        }

        paths = new Far9Paths(follower(), Robot.color);
        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("Circle for Blue, X for Red ");
        joinedTelemetry.addData("Current Team Color ", Robot.color);
    }


    @Override
    public void onStartButtonPressed() {
        follower().setStartingPose(Robot.color == Robot.teamColor.BLUE ? Poses.blueGoalTopStartFacing : Poses.redGoalTopStartFacing);
        Robot.INSTANCE.setGlobalColor();

        //Full auto
//        new SequentialGroup(
//                new ParallelGroup(
//                        TurretSubsystem.INSTANCE.SetFlywheelState(TurretSubsystem.FlywheelState.FAR),
//                        new FollowPath(paths.ShootPreload, true),
//                        new SequentialGroup(
//                                Robot.INSTANCE.TurretObelisk(),
//                                new WaitUntil(() -> Robot.patternFull),
//                                Robot.INSTANCE.TurretGoal()
//                        )
//                ),
//                //TODO: Shoot command
//                Robot.INSTANCE.ShootTest(),
//
//                IntakeSortingSubsystem.INSTANCE.runActive(),
//                new FollowPath(paths.GrabBottom, true),
//                //TODO: Shoot command
//                Robot.INSTANCE.ShootTest(),
//
//                new FollowPath(paths.GrabHuman, true),
//                //TODO: Shoot command
//                Robot.INSTANCE.ShootTest(),
//
//
//                new ParallelGroup(
//                        Robot.INSTANCE.StopSubsystems(),
//                        new FollowPath(paths.Park)
//                )
//        ).schedule();

        //Pathing
        new SequentialGroup(
                new FollowPath(paths.ShootPreload, true),
                new FollowPath(paths.GrabBottom, true),
                new FollowPath(paths.GrabHuman, true),
                new FollowPath(paths.Park)
        ).schedule();


    }

    @Override
    public void onUpdate() {
        Robot.order = IntakeSortingSubsystem.INSTANCE.determineOrder(Robot.pattern);
        endPose = follower().getPose();


    }

}


class Far9Paths {
    public PathChain ShootPreload;
    public PathChain GrabBottom;
    public PathChain GrabHuman;
    public PathChain Park;

    public Far9Paths(Follower follower, Robot.teamColor color) {

        if (color == Robot.teamColor.BLUE) {

            ShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(56.000, 16.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            GrabBottom = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 16.000),
                                    new Pose(63.880, 39.365),
                                    new Pose(14.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(14.000, 36.000),

                                    new Pose(56.000, 16.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();

            GrabHuman = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 16.000),

                                    new Pose(8.500, 8.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(8.500, 8.000),

                                    new Pose(56.000, 16.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 16.000),

                                    new Pose(39.018, 14.412)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .setNoDeceleration()
                    .build();
        } else {

            ShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    Poses.mirrorCoordinates(new Pose(56.000, 8.000), Robot.teamColor.RED),
                                    Poses.mirrorCoordinates(new Pose(56.000, 16.000), Robot.teamColor.RED)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            GrabBottom = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    Poses.mirrorCoordinates(new Pose(56.000, 16.000), Robot.teamColor.RED),
                                    Poses.mirrorCoordinates(new Pose(63.880, 39.365), Robot.teamColor.RED),
                                    Poses.mirrorCoordinates(new Pose(14.000, 36.000), Robot.teamColor.RED)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    Poses.mirrorCoordinates(new Pose(14.000, 36.000), Robot.teamColor.RED),

                                    Poses.mirrorCoordinates(new Pose(56.000, 16.000), Robot.teamColor.RED)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();

            GrabHuman = follower.pathBuilder().addPath(
                            new BezierLine(
                                    Poses.mirrorCoordinates(new Pose(56.000, 16.000), Robot.teamColor.RED),

                                    Poses.mirrorCoordinates(new Pose(8.500, 8.000), Robot.teamColor.RED)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    Poses.mirrorCoordinates(new Pose(8.500, 8.000), Robot.teamColor.RED),

                                    Poses.mirrorCoordinates(new Pose(56.000, 16.000), Robot.teamColor.RED)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    Poses.mirrorCoordinates(new Pose(56.000, 16.000), Robot.teamColor.RED),

                                    Poses.mirrorCoordinates(new Pose(39.018, 14.412), Robot.teamColor.RED)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))
                    .setNoDeceleration()
                    .build();
        }
    }
}

