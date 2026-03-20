package org.firstinspires.ftc.teamcode.OpModes;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class FarAuto extends NextFTCOpMode {

    {
        addComponents(
                new SubsystemComponent(RobotSubsystem.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private final Pose redStartingPose = new Pose(88, 8, Math.toRadians(90));
    private final Pose blueStartingPose = new Pose(16, 120, Math.toRadians(144));

    private final Pose redShootingPose = new Pose(87, 18, Math.toRadians(90));
    private final Pose blueShootingPose = new Pose(54, 84, Math.toRadians(150));



    private final Pose redIntakeCycleFinal = new Pose(136, 24, 0);
    private final Pose redIntakeCycleCP = new Pose(143.71518987341773, 2.0632911392404942);

    private final Pose redIntakeFarFinal = new Pose(128, 34, 0);
    private final Pose redIntakeFarCP = new Pose(74.30379746835442, 28.56962025316455);

    private final Pose redLeverPark = new Pose(120, 70);

    private final Pose redClosePark = new Pose(84, 134);

    private final Pose redLeverStrafeGoal = new Pose(160, 70);

    private PathChain startToShoot, shootToCycle, cycleToShoot, shootToClose, closeToShoot, shootToParkLever, shootToParkClose, shootToFar, farToShootCurved, farToShootLine, hitLeverFromMid, backFromLever, backFromMid;

    private final Timer pathTimer = new Timer();

    public static Pose prevPose = new Pose();

    boolean passed = false;

    private int oldState = -1;
    private int autoState = 0;

    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            RobotSubsystem.INSTANCE.setAllianceColor(RobotSubsystem.AllianceColor.RED);
        } else if (gamepad1.b) {
            RobotSubsystem.INSTANCE.setAllianceColor(RobotSubsystem.AllianceColor.BLUE);
        }

        telemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        telemetry.addLine("Circle for Blue, X for Red ");
        telemetry.addData("Current Team Color ", RobotSubsystem.INSTANCE.getAllianceColor());
    }

    @Override
    public void onStartButtonPressed(){
        OuttakeSubsystem.INSTANCE.resetTurret();
        RobotSubsystem.INSTANCE.resetPattern();
        UniConstants.FAST_FLICKER_TIME_UP = .325;
        PedroComponent.follower().setStartingPose(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redStartingPose : blueStartingPose);
        RobotSubsystem.inTele = false;
        createPaths();

        autoState = 1;
    }

    @Override
    public void onUpdate(){

        Pose currentPose = PedroComponent.follower().getPose();
        if(!currentPose.roughlyEquals(new Pose(0, 0), 10)){
            prevPose = PedroComponent.follower().getPose();

        }

        autoPathUpdate();
        telemetry.addData("Passed: ", passed);
        telemetry.addData("Auto State: ", autoState);
        telemetry.addData("Follower X: ", PedroComponent.follower().getPose().getX());
        telemetry.addData("Follower Y: ", PedroComponent.follower().getPose().getY());
        telemetry.addData("Follower H: ", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        OuttakeSubsystem.INSTANCE.sendTelemetry();
    }

    @Override
    public void onStop(){
        UniConstants.FAST_FLICKER_TIME_UP = .3;
    }

    private void autoPathUpdate(){
        switch (autoState){

            case 1:
                if(oldState != autoState){
                    new SequentialGroup(
                            new ParallelGroup(
                                    OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.GOAL),
                                    OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.REACTIVE),

                                    //new InstantCommand(() -> OuttakeSubsystem.hoodLinreg = false),
                                    new FollowPath(startToShoot)
                            ),
                            SetPassed(true),
                            new Delay(.25),
                            RobotSubsystem.INSTANCE.AutoShoot(),
                            SetAutoState(2)
                    ).schedule();
                }
                oldState = autoState;
                break;

        }
    }

    private void setAutoState(int state){
        if(state >= 0) {
            autoState = state;
        } else {
            autoState = -1;
        }
        pathTimer.reset();
    }

    public Command SetAutoState(int state){
        return new InstantCommand(() -> setAutoState(state));
    }

    public Command SetPassed(boolean pass){
        return new InstantCommand(() -> passed = pass);
    }

    public void createPaths(){
        startToShoot = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                Poses.mirrorCoordinates(redStartingPose, RobotSubsystem.INSTANCE.getAllianceColor()),
                                Poses.mirrorCoordinates(redShootingPose, RobotSubsystem.INSTANCE.getAllianceColor())
                        )
                )
                .setConstantHeadingInterpolation(
                        Poses.mirrorCoordinates(new Pose(0, 0, Math.toRadians(90)), RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                )
                .build();
        shootToFar = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                Poses.mirrorCoordinates(redShootingPose, RobotSubsystem.INSTANCE.getAllianceColor()),
                                Poses.mirrorCoordinates(redIntakeFarCP, RobotSubsystem.INSTANCE.getAllianceColor()),
                                Poses.mirrorCoordinates(redIntakeFarFinal, RobotSubsystem.INSTANCE.getAllianceColor())
                        )
                )
                .setConstantHeadingInterpolation(
                        Poses.mirrorCoordinates(redIntakeFarFinal, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                )
                .build();
        farToShootLine = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                Poses.mirrorCoordinates(redIntakeFarFinal, RobotSubsystem.INSTANCE.getAllianceColor()),
                                Poses.mirrorCoordinates(redShootingPose, RobotSubsystem.INSTANCE.getAllianceColor())
                        )
                )
                .setConstantHeadingInterpolation(
                        Poses.mirrorCoordinates(new Pose(0, 0, Math.toRadians(90)), RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                )
                .build();
        shootToCycle = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                Poses.mirrorCoordinates(redShootingPose, RobotSubsystem.INSTANCE.getAllianceColor()),
                                Poses.mirrorCoordinates(redIntakeCycleCP, RobotSubsystem.INSTANCE.getAllianceColor()),
                                Poses.mirrorCoordinates(redIntakeCycleFinal, RobotSubsystem.INSTANCE.getAllianceColor())
                        )
                )
                .setConstantHeadingInterpolation(
                        Poses.mirrorCoordinates(redIntakeCycleFinal, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                )
                .build();
        cycleToShoot = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                Poses.mirrorCoordinates(redIntakeCycleFinal, RobotSubsystem.INSTANCE.getAllianceColor()),
                                Poses.mirrorCoordinates(redShootingPose, RobotSubsystem.INSTANCE.getAllianceColor())
                        )
                )
                .setLinearHeadingInterpolation(
                        Poses.mirrorCoordinates(redIntakeCycleFinal, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading(),
                        Poses.mirrorCoordinates(new Pose(0, 0, Math.toRadians(90)), RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                )
                .build();


    }

}