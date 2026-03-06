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
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "FAr h")
public class FarHelper extends NextFTCOpMode {

    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(RobotSubsystem.INSTANCE)
        );
    }

    private int oldState = -1;
    private int autoState = 0;
    private int maxState = 0;
    private final Timer pathTimer = new Timer();

    private final RobotSubsystem.AutoSelect autoSelect = RobotSubsystem.AutoSelect.CLOSE;

    private Supplier<PathChain> parkClosePathSupplier;
    private Supplier<PathChain> shortShootPathSupplier;
    private Supplier<PathChain> shortParkShootPath;

    private Supplier<PathChain> closeIntakePath;
    private Supplier<PathChain> midIntakePath;
    private Supplier<PathChain> farIntakePath;

    private Supplier<PathChain> middleShootPath;
    private Supplier<PathChain> bumpLever;

    private Supplier<PathChain> farShootPath;
    private Supplier<PathChain> humanIntake;
    private Supplier<PathChain> farParkPath;

    private final double blueIntakeHeading = Math.toRadians(180);
    private final double redIntakeHeading = Math.toRadians(0);
    private static double intakeHeading = 0;

    private final double blueShootHeading = Math.toRadians(135);
    private final double redShootHeading = Math.toRadians(47);
    private static double shootHeading = 0;


    Command shootPath;
    private Command scorePreload;
    private Command intakeAndScoreClose;

    private Command intakeAndScoreMid;

    private int shortVelo = 2125;
    private double hoodTarget = .65;


    @Override
    public void onInit(){
        parkClosePathSupplier = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueClosePark : Poses.redClosePark)))
                .setNoDeceleration()
                .build();

        shortShootPathSupplier = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierCurve(PedroComponent.follower().getPose(), new Pose(76.26582278481013, 53.72784810126581), (RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueCloseAutoShoot : Poses.redCloseAutoShoot).minus(new Pose(0 ,5)))))
//                .setLinearHeadingInterpolation(PedroComponent.follower().getPose().getHeading(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? Math.toRadians(90) : Math.toRadians(130))
                .setConstantHeadingInterpolation(shootHeading)
                .build();
        shortParkShootPath = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower().getPose(), (RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueCloseAutoShoot : Poses.redCloseAutoShoot).plus(new Pose(0, 30)))))
//                .setLinearHeadingInterpolation(PedroComponent.follower().getPose().getHeading(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? Math.toRadians(90) : Math.toRadians(130))
                .setConstantHeadingInterpolation(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Math.toRadians(130) : Math.toRadians(95))
                .build();

        farIntakePath = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierCurve(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.farBlueIntakeControlPoint : Poses.farRedIntakeControlPoint, RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.farIntakeBlue : Poses.farIntakeRed)))
                .setConstantHeadingInterpolation(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? blueIntakeHeading : redIntakeHeading)
//                .setTangentHeadingInterpolation()
                .build();

        midIntakePath = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierCurve(PedroComponent.follower().getPose(),  RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.midBlueIntakeControlPoint : Poses.midRedIntakeControlPoint, RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.midIntakeBlue : Poses.midIntakeRed)))
                .setConstantHeadingInterpolation(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? blueIntakeHeading : redIntakeHeading)
//                .setTangentHeadingInterpolation()
                .build();

        closeIntakePath = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierCurve(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.closeBlueIntakeControlPoint : Poses.closeRedIntakeControlPoint, RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.closeIntakeBlue : Poses.closeIntakeRed)))
                .setConstantHeadingInterpolation(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? blueIntakeHeading : redIntakeHeading)
//                .setTangentHeadingInterpolation()
                .build();

        bumpLever = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueLever : Poses.redLever)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        farShootPath = () -> PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueFarShoot : Poses.redFarShoot)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        humanIntake = () -> PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? new Pose(135, 9) : Poses.mirrorCoordinates(new Pose(135, 9), RobotSubsystem.AllianceColor.BLUE))))
                .setConstantHeadingInterpolation(intakeHeading)
                .build();

        farParkPath = () -> PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? new Pose(105, 32) : Poses.mirrorCoordinates(new Pose(105, 32), RobotSubsystem.AllianceColor.BLUE))))
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


        telemetry.addData("Current Auto: ", autoSelect);

    }

    @Override
    public void onStartButtonPressed() {
        RobotSubsystem.inTele = false;
        RobotSubsystem.INSTANCE.resetPattern();
        OuttakeSubsystem.INSTANCE.resetTurret();
        shootHeading = RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? blueShootHeading : redShootHeading;
        shootPath = new FollowPath(farShootPath.get());
        intakeHeading = RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? blueIntakeHeading : redIntakeHeading;
        PedroComponent.follower().setStartingPose(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueFarStart : Poses.redFarStart);
        maxState = 4;
        scorePreload = new SequentialGroup(
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                new ParallelGroup(
                        new SequentialGroup(
                                OuttakeSubsystem.INSTANCE.ScanPattern(),
                                OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.GOAL)
                        ),
                        OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.INTERPOLATED),
                        shootPath
                ),
                //new TurnTo(Angle.fromRad(Math.toRadians(48))),
                new Delay(3),
                RobotSubsystem.INSTANCE.AutoShoot(),
                SetAutoState(1)
        );
        intakeAndScoreClose = new SequentialGroup(
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                new ParallelDeadlineGroup(new Delay(2.5), new FollowPath(farIntakePath.get())),
//                            new FollowPath(bumpLever.get()),
                shootPath,
                //new TurnTo(Angle.fromRad(Math.toRadians(48))),
                OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.GOAL),
                new Delay(.5),
                RobotSubsystem.INSTANCE.AutoShoot(),
                SetAutoState(2)
        );
        intakeAndScoreMid = new SequentialGroup(
                //new TurnTo(Angle.fromRad(intakeHeading)),
                new FollowPath(midIntakePath.get()),
//                            new ParallelDeadlineGroup(new Delay(.8), new FollowPath(bumpLever.get())),
                new FollowPath(new Path(new BezierLine(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueLever.plus(new Pose(20, 0)) : Poses.mirrorCoordinates(Poses.blueLever.plus(new Pose(20, 0)), RobotSubsystem.AllianceColor.RED)))),
                new FollowPath(bumpLever.get()),
                shootPath,
                //new TurnTo(Angle.fromRad(Math.toRadians(48))),
                OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.GOAL),
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OUT),
                RobotSubsystem.INSTANCE.AutoShoot(),
                SetAutoState(2)
        );
//        OuttakeSubsystem.hoodLinreg = false;
//        OuttakeSubsystem.INSTANCE.setHood(hoodTarget);
//        OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.LAZY).schedule();
//        OuttakeSubsystem.lazyRPM = 2200;
        setAutoState(0);
    }


    @Override
    public void onUpdate(){
        Tele.autoPose = PedroComponent.follower().getPose();

        autoPathUpdate();
        OuttakeSubsystem.INSTANCE.sendTelemetry();
        //telemetry.addData("Command Manager: ", CommandManager.INSTANCE.snapshot());
        telemetry.addData("Pose X: ", PedroComponent.follower().getPose().getX());
        telemetry.addData("Pose Y: ", PedroComponent.follower().getPose().getY());
        telemetry.addData("Pose Head: ", PedroComponent.follower().getPose().getHeading());
        //telemetry.addData("Distance to Goal: ", RobotSubsystem.INSTANCE.getDistanceToGoalInches());
        telemetry.addData("Path State: ", autoState);
        telemetry.addData("Old State: ", oldState);
        if(PedroComponent.follower().getCurrentPath() != null) {
            telemetry.addData("Goal X: ", PedroComponent.follower().getCurrentPath().getLastControlPoint().getX());
            telemetry.addData("Goal Y: ", PedroComponent.follower().getCurrentPath().getLastControlPoint().getX());
            telemetry.addData("Goal Heading: ", PedroComponent.follower().getCurrentPath().getLastControlPoint().getHeading());
        }
    }


    private void setAutoState(int state){
        if(state <= maxState && state >= 0) {
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
                    oldState = autoState;
                    scorePreload.schedule();
                }
                break;
            case 1:
                if(oldState != autoState){
                    oldState = autoState;
                    intakeAndScoreClose.schedule();
                }
                break;
            case 2:
                if(oldState != autoState){
                    oldState = autoState;
                    intakeAndScoreMid.schedule();
                }
                break;
//            case 3:
//                if(oldState != autoState){
//                    oldState = autoState;
//                    new SequentialGroup(
//                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
//                            new ParallelDeadlineGroup(new Delay(1.5), new FollowPath(farIntakePath.get())),
//                            new FollowPath(shortParkShootPath.get()),
//                            new Delay(.5),
//                            RobotSubsystem.INSTANCE.AutoShoot(),
//                            SetAutoState(4)
//                    ).schedule();
//                }
//                break;
            case 4:
                if(oldState != autoState){
                    oldState = autoState;
                    new ParallelGroup(
                            RobotSubsystem.INSTANCE.stopSubsystems(),
                            new FollowPath(farParkPath.get()),
                            SetAutoState(-1)
                    ).schedule();
                }
                break;



        }
    }




}
