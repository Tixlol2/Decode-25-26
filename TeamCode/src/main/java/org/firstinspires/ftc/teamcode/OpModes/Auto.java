package org.firstinspires.ftc.teamcode.OpModes;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.AutonUtil.Short9BallPaths;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSortingSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.function.Supplier;

import dev.nextftc.core.commands.delays.Delay;
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

@Autonomous(name = "Auto", group = "Main") //The name and group
@Configurable
public class Auto extends NextFTCOpMode {
    public static Pose endPose = new Pose();
    JoinedTelemetry joinedTelemetry;

//    public static Pose startPose = Poses.blueGoalTopStartFacing;
    Supplier<PathChain> shootPath;
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

        shootPath = () -> follower().pathBuilder()
                .addPath(new Path(new BezierLine(follower()::getPose, Robot.color == Robot.teamColor.BLUE ? Poses.blueShortScore : Poses.redShortScore)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower()::getHeading, Robot.color == Robot.teamColor.BLUE ? Poses.blueShortScore.getHeading() : Poses.redShortScore.getHeading(), 0.8)).build();

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

        new SequentialGroup(
                TurretSubsystem.INSTANCE.SetMotorPower(.625),
                new ParallelGroup(
                        Robot.INSTANCE.TurretObelisk(),
                        new FollowPath(paths.StartShoot),
                        new Delay(4) //For outtake
                ),
                new InstantCommand(() -> {
                    Robot.order = IntakeSortingSubsystem.INSTANCE.determineOrder(Robot.pattern);
                }),

                Robot.INSTANCE.TurretGoal(),

                IntakeSortingSubsystem.INSTANCE.Shoot(),
                new FollowPath(paths.ReadyIntakeTop),
                new ParallelGroup(
                        new FollowPath(paths.IntakeTop),
                        IntakeSortingSubsystem.INSTANCE.runActive()
                ),
//                Robot.INSTANCE.ActivePath(paths.IntakeTop, false, .75),
                new FollowPath(paths.TopShoot, true),
                IntakeSortingSubsystem.INSTANCE.Shoot(),
                IntakeSortingSubsystem.INSTANCE.stopActive(),
                new FollowPath(paths.ReadyIntakeMid),
                new ParallelGroup(
                        new FollowPath(paths.IntakeMid),
                        IntakeSortingSubsystem.INSTANCE.runActive()
                ),
//                Robot.INSTANCE.ActivePath(paths.IntakeMid, false, .75),
                new FollowPath(paths.MidShoot),
                IntakeSortingSubsystem.INSTANCE.Shoot(),
                new ParallelGroup(
                        new FollowPath(paths.Park),
                        Robot.INSTANCE.StopSubsystems()
                )
        ).schedule();
    }

    @Override
    public void onUpdate() {
        Robot.order = IntakeSortingSubsystem.INSTANCE.determineOrder(Robot.pattern);
        endPose = follower().getPose();
    }

}
