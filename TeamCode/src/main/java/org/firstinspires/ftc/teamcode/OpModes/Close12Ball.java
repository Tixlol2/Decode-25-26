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

@Autonomous(name = "Close 12", group = "Main") //The name and group
@Configurable
public class Close12Ball extends NextFTCOpMode {
    public static Pose endPose = new Pose();
    JoinedTelemetry joinedTelemetry;

//    public static Pose startPose = Poses.blueGoalTopStartFacing;
    Close12Paths paths;

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

        paths = new Close12Paths(follower(), Robot.color);
        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("Circle for Blue, X for Red ");
        joinedTelemetry.addData("Current Team Color ", Robot.color);
    }


    @Override
    public void onStartButtonPressed() {
        follower().setStartingPose(Robot.color == Robot.teamColor.BLUE ? Poses.blueGoalTopStartFacing : Poses.redGoalTopStartFacing);
        Robot.INSTANCE.setGlobalColor();

        //FULL AUTO
//        new SequentialGroup(
//                new ParallelGroup(
//                        TurretSubsystem.INSTANCE.SetFlywheelState(TurretSubsystem.FlywheelState.SHORT),
//                        new FollowPath(paths.ObeliskShoot, true),
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
//                new FollowPath(paths.IntakeMidLeverTurnAtEnd),
//                IntakeSortingSubsystem.INSTANCE.stopActive(),
//                new TurnTo(Angle.fromDeg(90)),
//                new FollowPath(paths.ShootMid, true),
//                //TODO: Shoot command
//                Robot.INSTANCE.ShootTest(),
//
//                IntakeSortingSubsystem.INSTANCE.runActive(),
//                new FollowPath(paths.Top, true),
//                //TODO: Shoot command
//                Robot.INSTANCE.ShootTest(),
//
//                new FollowPath(paths.Bottom, true),
//                //TODO: Shoot command
//                Robot.INSTANCE.ShootTest(),
//
//                new ParallelGroup(
//                        Robot.INSTANCE.StopSubsystems(),
//                        new FollowPath(paths.Park)
//                )
//        ).schedule();

        //Pathing
        new SequentialGroup(
                new FollowPath(paths.ObeliskShoot, true),
                new FollowPath(paths.IntakeMidLeverTurnAtEnd),
                new FollowPath(paths.ShootMid, true),
                new FollowPath(paths.Top, true),
                new FollowPath(paths.Bottom, true),
                new FollowPath(paths.Park)
        ).schedule();


    }

    @Override
    public void onUpdate() {
        Robot.order = IntakeSortingSubsystem.INSTANCE.determineOrder(Robot.pattern);
        endPose = follower().getPose();


    }

    static class Close12Paths {
        public PathChain ObeliskShoot;
        public PathChain IntakeMidLeverTurnAtEnd;
        public PathChain ShootMid;
        public PathChain Top;
        public PathChain Bottom;
        public PathChain Park;

        public Close12Paths(Follower follower, Robot.teamColor color) {

            if(color == Robot.teamColor.BLUE){

                ObeliskShoot = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(22.000, 124.000),
                                        new Pose(48.000, 96.000)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(144))

                        .build();

                IntakeMidLeverTurnAtEnd = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(48.000, 96.000),
                                        new Pose(59.523, 54.254),
                                        new Pose(16.000, 60.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                        .build();

                ShootMid = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(16.000, 60.000),
                                        new Pose(9.482, 77.487),
                                        new Pose(48.000, 96.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                        .build();

                Top = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(48.000, 96.000),
                                        new Pose(38.433, 81.344),
                                        new Pose(16.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                        .addPath(
                                new BezierLine(
                                        new Pose(16.000, 84.000),

                                        new Pose(48.000, 96.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))

                        .build();

                Bottom = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(48.000, 96.000),
                                        new Pose(69.423, 30.853),
                                        new Pose(16.000, 36.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                        .addPath(
                                new BezierLine(
                                        new Pose(16.000, 36.000),
                                        new Pose(48.000, 96.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))

                        .build();


                Park = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(48.000, 96.000),
                                        new Pose(48.000, 120.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(90))
                        .setNoDeceleration()
                        .build();


        } else {


                ObeliskShoot = follower.pathBuilder().addPath(
                                new BezierLine(
                                        Poses.mirrorCoordinates(new Pose(22.000, 124.000), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(48.000, 96.000), Robot.teamColor.RED)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(37))

                        .build();

                IntakeMidLeverTurnAtEnd = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        Poses.mirrorCoordinates(new Pose(48.000, 96.000), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(59.523, 54.254), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(16.000, 60.000), Robot.teamColor.RED)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                        .build();

                ShootMid = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        Poses.mirrorCoordinates(new Pose(16.000, 60.000), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(9.482, 77.487), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(48.000, 96.000), Robot.teamColor.RED)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                        .build();

                Top = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        Poses.mirrorCoordinates(new Pose(48.000, 96.000), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(38.433, 81.344), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(16.000, 84.000), Robot.teamColor.RED)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        Poses.mirrorCoordinates(new Pose(16.000, 84.000), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(48.000, 96.000), Robot.teamColor.RED)

                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))

                        .build();

                Bottom = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        Poses.mirrorCoordinates(new Pose(48.000, 96.000), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(69.423, 30.853), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(16.000, 36.000), Robot.teamColor.RED)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                        .addPath(
                                new BezierLine(
                                        Poses.mirrorCoordinates(new Pose(16.000, 36.000), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(48.000, 96.000), Robot.teamColor.RED)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))

                        .build();


                Park = follower.pathBuilder().addPath(
                                new BezierLine(
                                        Poses.mirrorCoordinates(new Pose(48.000, 96.000), Robot.teamColor.RED),
                                        Poses.mirrorCoordinates(new Pose(48.000, 120), Robot.teamColor.RED)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(90))
                        .setNoDeceleration()
                        .build();

            }
        }

    }
}


