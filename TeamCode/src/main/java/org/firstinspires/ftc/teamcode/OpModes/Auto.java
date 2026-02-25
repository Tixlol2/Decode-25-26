package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.Util.IfElseCommand;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class Auto extends NextFTCOpMode {

    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(RobotSubsystem.INSTANCE)
        );
    }

    private int oldState = 0;
    private int autoState = 0;
    private int maxState = 0;
    private final Timer pathTimer = new Timer();

    private AutoSelect autoSelect = AutoSelect.CLOSE;

    private Supplier<PathChain> parkClosePathSupplier;
    private Supplier<PathChain> shortShootPathSupplier;
    private Supplier<PathChain> farShootPathSupplier;

    private Supplier<PathChain> closeIntakePath;
    private Supplier<PathChain> midIntakePath;
    private Supplier<PathChain> farIntakePath;

    private Supplier<PathChain> bumpLever;

    private double blueIntakeHeading = Math.toRadians(180);
    private double redIntakeHeading = Math.toRadians(0);


    Command shootPath = new IfElseCommand(() -> autoSelect.equals(AutoSelect.CLOSE), new FollowPath(shortShootPathSupplier.get()), new FollowPath(farShootPathSupplier.get()));


    @Override
    public void onInit(){
        parkClosePathSupplier = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueClosePark : Poses.redClosePark)))
                .setNoDeceleration()
                .build();

        shortShootPathSupplier = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueCloseAutoShoot : Poses.redCloseAutoShoot)))
                .setTangentHeadingInterpolation()
                .build();

        farShootPathSupplier = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueFarAutoShoot : Poses.redFarAutoShoot)))
                .setLinearHeadingInterpolation(PedroComponent.follower().getPose().getHeading(), Math.toRadians(90))
                .build();

        farIntakePath = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierCurve(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.farBlueIntakeControlPoint : Poses.farRedIntakeControlPoint, RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.farIntakeBlue : Poses.farIntakeRed)))
                .setConstantHeadingInterpolation(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? blueIntakeHeading : redIntakeHeading)
                .build();

        midIntakePath = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierCurve(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.midBlueIntakeControlPoint : Poses.midRedIntakeControlPoint, RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.midIntakeBlue : Poses.midIntakeRed)))
                .setConstantHeadingInterpolation(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? blueIntakeHeading : redIntakeHeading)
                .build();

        closeIntakePath = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierCurve(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.closeBlueIntakeControlPoint : Poses.closeRedIntakeControlPoint, RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.closeIntakeBlue : Poses.closeIntakeRed)))
                .setConstantHeadingInterpolation(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? blueIntakeHeading : redIntakeHeading)
                .build();

        bumpLever = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierCurve(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueLeverBumpControlPoint : Poses.redLeverBumpControlPoint, RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueLeverBump : Poses.redLeverBump)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    @Override
    public void onWaitForStart(){

        if (gamepad1.a) {
            RobotSubsystem.INSTANCE.setAllianceColor(RobotSubsystem.AllianceColor.RED);
        } else if (gamepad1.b) {
            RobotSubsystem.INSTANCE.setAllianceColor(RobotSubsystem.AllianceColor.BLUE);
        }
        telemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        telemetry.addLine("Circle for Blue, X for Red ");
        telemetry.addData("Current Team Color ", RobotSubsystem.INSTANCE.getAllianceColor());
        telemetry.addLine();

        if(gamepad1.dpad_up){
            autoSelect = AutoSelect.CLOSE;
        } else if (gamepad1.dpad_down){
            autoSelect = AutoSelect.FAR;
        }

        telemetry.addData("Current Auto: ", autoSelect);

    }

    @Override
    public void onStartButtonPressed() {
        switch (autoSelect) {
            case CLOSE:
                PedroComponent.follower().setStartingPose(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueCloseStart : Poses.redCloseStart);
                break;
            case FAR:
                PedroComponent.follower().setStartingPose(new Pose());
                break;
        }
        setAutoState(0);
    }


    @Override
    public void onUpdate(){
        autoPathUpdate();
    }

    private void setAutoState(int state){
        if(state < maxState && state >= 0) {
            autoState = state;
        } else {
            autoState = -1;
        }
        pathTimer.reset();
    }

    public Command SetAutoState(int state){
        return new InstantCommand(() -> setAutoState(state));
    }

    private void autoPathUpdate(){
        switch (autoState){

            case -1:
                break;
            case 0:
                if(oldState != autoState) {
                    new SequentialGroup(
                            OuttakeSubsystem.INSTANCE.ScanPattern(),
                            OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.INTERPOLATED),
                            shootPath,
                            RobotSubsystem.INSTANCE.Shoot(),
                            SetAutoState(1)
                    ).schedule();
                    oldState = autoState;
                }
                break;
            case 1:
                if(oldState != autoState) {
                    new SequentialGroup(
                            new ParallelRaceGroup(
                                    new IfElseCommand(() -> autoSelect == AutoSelect.CLOSE, new FollowPath(closeIntakePath.get()), new FollowPath(farIntakePath.get())),
                                    IntakeSubsystem.INSTANCE.runActive()
                            ),
                            shootPath,
                            RobotSubsystem.INSTANCE.Shoot(),
                            SetAutoState(2)
                            ).schedule();
                    oldState = autoState;
                }
                break;
            case 2:
                if(oldState != autoState) {
                    new SequentialGroup(
                            new ParallelRaceGroup(
                                    new IfElseCommand(() -> autoSelect == AutoSelect.CLOSE, new FollowPath(midIntakePath.get()), new FollowPath(midIntakePath.get())),
                                    IntakeSubsystem.INSTANCE.runActive()
                            ),
                            shootPath,
                            RobotSubsystem.INSTANCE.Shoot(),
                            SetAutoState(3)
                    ).schedule();
                    oldState = autoState;
                }
            case 3:
                new FollowPath(parkClosePathSupplier.get()).schedule();
                setAutoState(-1);
                break;

        }
    }



    enum AutoSelect{
        CLOSE,
        FAR
    }


}
