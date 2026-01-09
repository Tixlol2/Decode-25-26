package org.firstinspires.ftc.teamcode.OpModes;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.AutonUtil.Short9BallPaths;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSortingSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.function.Supplier;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
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
    JoinedTelemetry joinedTelemetry;



    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        ); //Subsystems
    }

//    public static Pose startPose = Poses.blueGoalTopStartFacing;

    Supplier<PathChain> shootPath;




    Short9BallPaths paths;


    @Override
    public void onInit() {
        joinedTelemetry = Robot.INSTANCE.getJoinedTelemetry();

        paths = new Short9BallPaths(follower(), Robot.color);

        shootPath = () -> follower().pathBuilder()
                .addPath(new Path(new BezierLine(follower()::getPose, Robot.color == UniConstants.teamColor.BLUE ? Poses.blueShortScore : Poses.redShortScore)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower()::getHeading, Robot.color == UniConstants.teamColor.BLUE ? Poses.blueShortScore.getHeading() : Poses.redShortScore.getHeading(), 0.8)).build();

    }

    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            Robot.color = UniConstants.teamColor.RED;
            paths = new Short9BallPaths(follower(), Robot.color);
        } else if (gamepad1.b) {
            Robot.color = UniConstants.teamColor.BLUE;
            paths = new Short9BallPaths(follower(), Robot.color);
        }

        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("Circle for Blue, X for Red ");
        joinedTelemetry.addData("Current Team Color ", Robot.color);
    }


    @Override
    public void onStartButtonPressed() {
        follower().setStartingPose(Robot.color == UniConstants.teamColor.BLUE ? Poses.blueGoalTopStartFacing : Poses.redGoalTopStartFacing);
        Robot.INSTANCE.setGlobalColor();
        Robot.inTeleop = false;

        new SequentialGroup(
                TurretSubsystem.INSTANCE.SetMotorPower(.65),
                new ParallelGroup(
                        Robot.INSTANCE.TurretObelisk(),
                        new FollowPath(paths.Path1),
                        new Delay(3) //For outtake
                ),
                Robot.INSTANCE.TurretForward(),
                Robot.INSTANCE.ShootWait(.25),
                new FollowPath(paths.Path2),
                Robot.INSTANCE.ActivePath(paths.Path3, false, 1),
                Robot.INSTANCE.PathShoot(),
                IntakeSortingSubsystem.INSTANCE.stopActive(),
                new FollowPath(paths.Path4),
                Robot.INSTANCE.ActivePath(paths.Path5, false, 1),
                Robot.INSTANCE.PathShoot(),
                new ParallelGroup(
                        new FollowPath(paths.Path6),
                        Robot.INSTANCE.StopSubsystems()
                )
        ).schedule();
    }

    @Override
    public void onUpdate() {

    }

}
