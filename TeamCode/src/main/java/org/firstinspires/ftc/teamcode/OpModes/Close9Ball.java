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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

//Written by Noah Nottingham - 6566 Circuit Breakers

@Autonomous(name = "Close 9", group = "Main") //The name and group
@Configurable
public class Close9Ball extends NextFTCOpMode {
    public static Pose endPose = new Pose();
    JoinedTelemetry joinedTelemetry;

    //    public static Pose startPose = Poses.blueGoalTopStartFacing;
    int pathState = 0;
    int oldPathState = 0;
    Short9BallPaths paths;

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
        paths = new Short9BallPaths(follower(), Robot.color);

    }

    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            Robot.color = Robot.teamColor.RED;
            paths = new Short9BallPaths(follower(), Robot.color);
        } else if (gamepad1.b) {
            Robot.color = Robot.teamColor.BLUE;
            paths = new Short9BallPaths(follower(), Robot.color);
        }

        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("Circle for Blue, X for Red ");
        joinedTelemetry.addData("Current Team Color ", Robot.color);
    }


    @Override
    public void onStartButtonPressed() {
        follower().setStartingPose(Robot.color == Robot.teamColor.BLUE ? Poses.blueGoalTopStartFacing : Poses.redGoalTopStartFacing);
        Robot.INSTANCE.setGlobalColor();
        setPathState(1);
    }

    @Override
    public void onUpdate() {
        endPose = follower().getPose();
        if (pathState != oldPathState) {
            oldPathState = pathState;
            pathUpdate();
        }

        joinedTelemetry.addData("Path state: ", pathState);
        joinedTelemetry.addData("Commands: ", CommandManager.INSTANCE.snapshot());

    }

    public void setPathState(int state) {
        pathState = state;
    }

    public Command SetPathState(int state) {
        return new InstantCommand(() -> setPathState(state));
    }

    public void pathUpdate() {

        switch (pathState) {
            case 0:
                break;
            case 1:
                new SequentialGroup(
                        TurretSubsystem.INSTANCE.SetFlywheelState(TurretSubsystem.FlywheelState.SHORT),
                        new ParallelGroup(
                                new SequentialGroup(
                                        TurretSubsystem.INSTANCE.TurretObelisk(),
                                        new WaitUntil(() -> Robot.patternFull),
                                        TurretSubsystem.INSTANCE.TurretGoal()
                                ),
                                new FollowPath(paths.StartShoot)
                        ),

                        IntakeSortingSubsystem.INSTANCE.Shoot(),
                        IntakeSortingSubsystem.INSTANCE.SetAllSlotState(IntakeSortingSubsystem.servoState.DOWN),
                        SetPathState(2)
                ).schedule();
                break;
            case 2:
                new SequentialGroup(
                        new FollowPath(paths.ReadyIntakeTop),
                        new ParallelGroup(
                                new FollowPath(paths.IntakeTop),
                                IntakeSortingSubsystem.INSTANCE.runActive()
                        ),
                        IntakeSortingSubsystem.INSTANCE.reverseActive(),
//                Robot.INSTANCE.ActivePath(paths.IntakeTop, false, .75),
                        new FollowPath(paths.TopShoot, true),
                        IntakeSortingSubsystem.INSTANCE.Shoot(),
                        IntakeSortingSubsystem.INSTANCE.SetAllSlotState(IntakeSortingSubsystem.servoState.DOWN),
                        IntakeSortingSubsystem.INSTANCE.stopActive(),
                        SetPathState(3)
                ).schedule();
                break;
            case 3:
                new SequentialGroup(
                        new FollowPath(paths.ReadyIntakeMid),
                        new ParallelGroup(
                                new FollowPath(paths.IntakeMid),
                                IntakeSortingSubsystem.INSTANCE.runActive()
                        ),
                        IntakeSortingSubsystem.INSTANCE.reverseActive(),

//                Robot.INSTANCE.ActivePath(paths.IntakeMid, false, .75),
                        new FollowPath(paths.MidShoot, true),
                        IntakeSortingSubsystem.INSTANCE.Shoot(),
                        IntakeSortingSubsystem.INSTANCE.SetAllSlotState(IntakeSortingSubsystem.servoState.DOWN),
                        SetPathState(4)
                ).schedule();
                break;
            case 4:
                new ParallelGroup(
                        new FollowPath(paths.Park),
                        Robot.INSTANCE.StopSubsystems()
                )
                        .schedule();
                break;
        }


    }

    public static class Short9BallPaths {

        public PathChain StartShoot;
        public PathChain ReadyIntakeTop;
        public PathChain IntakeTop;
        public PathChain TopShoot;

        public PathChain ReadyIntakeMid;
        public PathChain IntakeMid;
        public PathChain MidShoot;

        public PathChain Park;

        public Short9BallPaths(Follower follower, Robot.teamColor color) {
            if (color == Robot.teamColor.BLUE) {
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
                    .addPath(new BezierLine(Poses.redActiveTopStop, Poses.redShortScore))
                    .setConstantHeadingInterpolation(Math.toRadians(37))
                    .build();
        }

        public void blueShort(Follower follower) {
            StartShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.blueGoalTopStartFacing, Poses.blueShortScore)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(75))
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
                    //.setNoDeceleration()
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
                    .addPath(new BezierCurve(Poses.blueActiveMidStop, Poses.blueMidCP, Poses.blueShortScore))
                    .setConstantHeadingInterpolation(Math.toRadians(144))
                    .build();

        }
    }
}
