package org.firstinspires.ftc.teamcode.OpModes.Paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.SlotSubsystem;

import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;

public class MainPaths {

    public String name;
    public int numCommands;
    public ShootingLocation shootingLocation;
    public Pose startingPose;
    public Command command1;
    public Command command2;
    public Command command3;
    public Command command4;
    public Command command5;
    public Command command6;
    public Command command7;

    public MainPaths(String name, int numCommands, ShootingLocation shootingLocation, Pose startingPose) {
        this.name = name;
        this.numCommands = numCommands;
        this.shootingLocation = shootingLocation;
        this.startingPose = startingPose;
    }

    public void setCommand(int num, Command command) {
        switch (num) {
            case 1:
                command1 = command;
                break;
            case 2:
                command2 = command;
                break;
            case 3:
                command3 = command;
                break;
            case 4:
                command4 = command;
                break;
            case 5:
                command5 = command;
                break;
            case 6:
                command6 = command;
                break;
            case 7:
                command7 = command;
        }
    }

    public void setCommands(Command command1, Command command2, Command command3, Command command4, Command command5, Command command6, Command command7) {
        this.command1 = command1;
        this.command2 = command2;
        this.command3 = command3;
        this.command4 = command4;
        this.command5 = command5;
        this.command6 = command6;
        this.command7 = command7;
    }


    public PathChain getShootingPath() {
        Supplier<PathChain> path = () -> PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                PedroComponent.follower().getPose(),
                                shootingLocation == ShootingLocation.SHORT ? (Robot.color == Robot.teamColor.BLUE ? Poses.blueShortScore : Poses.redShortScore) : (Robot.color == Robot.teamColor.BLUE ? Poses.blueFarScore : Poses.redFarScore)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        return path.get();
    }

    public PathChain getParkPath() {
        Supplier<PathChain> path = () -> PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                PedroComponent.follower().getPose(),
                                shootingLocation == ShootingLocation.SHORT ? (Robot.color == Robot.teamColor.BLUE ? Poses.blueShortParkAuto : Poses.redShortParkAuto) : (Robot.color == Robot.teamColor.BLUE ? Poses.blueFarParkAuto : Poses.redFarParkAuto)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        return path.get();
    }

    public PathChain getTopIntakePath() {
        Supplier<PathChain> topIntakePath = () -> PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(
                        PedroComponent.follower().getPose(),
                        Robot.color == Robot.teamColor.BLUE ? new Pose(83.18323, 84) : Poses.mirrorCoordinates(new Pose(83.18323, 84), Robot.teamColor.RED),
                        Robot.color == Robot.teamColor.BLUE ? Poses.blueActiveTopStop : Poses.redActiveTopStop
                ))
                .setConstantHeadingInterpolation(Math.toRadians(Robot.color == Robot.teamColor.BLUE ? 180 : 0))
                .build();
        return topIntakePath.get();
    }

    public PathChain getMidIntakePath() {
        Supplier<PathChain> midIntakePath = () -> PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(
                        PedroComponent.follower().getPose(),
                        Robot.color == Robot.teamColor.BLUE ? Poses.mirrorCoordinates(new Pose(82.66586473053893, 58.994011976047915), Robot.teamColor.BLUE) : new Pose(82.66586473053893, 58.994011976047915),
                        Robot.color == Robot.teamColor.BLUE ? Poses.blueActiveMidStop : Poses.redActiveMidStop
                ))
                .setConstantHeadingInterpolation(Math.toRadians(Robot.color == Robot.teamColor.BLUE ? 180 : 0))
                .build();
        return midIntakePath.get();
    }

    public Command MoveShoot() {
        return new ParallelGroup(
                new FollowPath(getShootingPath()),
                SlotSubsystem.INSTANCE.Shoot()
        );
    }

    public Command Park() {
        return new ParallelGroup(
                new FollowPath(getParkPath()),
                Robot.INSTANCE.StopSubsystems()
        );
    }

    public Command ActivePath(PathChain path) {
        return new ParallelGroup(
                new FollowPath(path),
                IntakeSubsystem.INSTANCE.runActive()
        );
    }

    public Command IntakeTop() {
        return ActivePath(getTopIntakePath());
    }
    public Command IntakeMid() {
        return ActivePath(getMidIntakePath());
    }

    public enum ShootingLocation {
        SHORT,
        FAR
    }

}
