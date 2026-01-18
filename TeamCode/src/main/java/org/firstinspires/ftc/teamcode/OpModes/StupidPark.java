package org.firstinspires.ftc.teamcode.OpModes;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

//Written by Noah Nottingham - 6566 Circuit Breakers

@Autonomous(name = "Close 3", group = "Main") //The name and group
@Configurable
public class StupidPark extends NextFTCOpMode {
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
        setPathState(4);
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

        public PathChain Path1;
        public PathChain Path2;

        public PathChain Park;

        public Short9BallPaths(Follower follower, Robot.teamColor color) {
            if (color == Robot.teamColor.BLUE) {
                blueShort(follower);
            } else {
                redShort(follower);
            }
        }

        public void redShort(Follower follower) {

            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.redGoalTopStartFacing, Poses.mirrorCoordinates(new Pose(72.000, 133.000), Robot.teamColor.RED))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 133.000), new Pose(48.000, 133.000))
                    )
                    .setTangentHeadingInterpolation()
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


            Park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.blueGoalTopStartFacing, Poses.blueParkAuto)
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setNoDeceleration()
                    .build();


        }
    }
}
