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
import dev.nextftc.core.commands.CommandManager;
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

//9 Ball Far
@Autonomous(name = "Cycling Far + Spike", group = "Auto")
public class FarAuto extends NextFTCOpMode {

    {
        addComponents(
                new SubsystemComponent(RobotSubsystem.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private final Pose redStartingPose = new Pose(88, 8, Math.toRadians(0));
    private final Pose redShootingPose = new Pose(96, 11, Math.toRadians(0));


    private final Pose redIntakeCycleFinal = new Pose(127, 40, 0);
    private final Pose redIntakeCycleCP = new Pose(131.0506329113924, 5.037974683544299);

    private final Pose redIntakeFarFinal = new Pose(127, 35.01265822784809, 0);
    private final Pose redIntakeFarCP = new Pose(91.41772151898732, 40.126582278481);

    private final Pose redPark = new Pose(120, 15);


    private PathChain startToShoot, shootToCycle, cycleToShoot, shootToClose, closeToShoot, shootToParkLever, shootToParkClose, shootToFar, farToShootCurved, farToShootLine, hitLeverFromMid, backFromLever, backFromMid;

    private final Timer pathTimer = new Timer();

    public static Pose prevPose = new Pose();

    boolean passed = false;

    private int oldState = -1;
    private int autoState = 0;

    private Timer autoTimer = new Timer();
    private boolean cycle = true;
    private boolean killCycling = false;

    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            RobotSubsystem.INSTANCE.setAllianceColor(RobotSubsystem.AllianceColor.RED);
        } else if (gamepad1.b) {
            RobotSubsystem.INSTANCE.setAllianceColor(RobotSubsystem.AllianceColor.BLUE);
        }

        if (gamepad1.x) {
            cycle = true;
        } else if (gamepad1.y) {
            cycle = false;

        }

        telemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        telemetry.addLine("Circle for Blue, X for Red ");
        telemetry.addData("Current Team Color ", RobotSubsystem.INSTANCE.getAllianceColor());
        telemetry.addLine("X for yes, Y for no");
        telemetry.addData("Cycling? ", cycle);
    }

    @Override
    public void onStartButtonPressed() {
        OuttakeSubsystem.INSTANCE.resetTurret();
        RobotSubsystem.INSTANCE.resetPattern();
        UniConstants.FAST_FLICKER_TIME_UP = .225;
        PedroComponent.follower().setStartingPose(Poses.mirrorCoordinates(redStartingPose, RobotSubsystem.INSTANCE.getAllianceColor()));//RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redStartingPose : blueStartingPose);
        RobotSubsystem.inTele = false;
        RobotSubsystem.autoEnd = RobotSubsystem.AutoEnd.FAR;
        RobotSubsystem.INSTANCE.updatingDist = true;
        createPaths();

        autoState = 1;
        autoTimer.reset();
    }

    @Override
    public void onUpdate() {

        Pose currentPose = PedroComponent.follower().getPose();
        if (!currentPose.roughlyEquals(new Pose(0, 0), 10)) {
            prevPose = PedroComponent.follower().getPose();
        }

        if (!RobotSubsystem.INSTANCE.allSlotsEmpty() && autoTimer.getTimeSeconds() > 25 && !killCycling) {
            CommandManager.INSTANCE.cancelAll();
            new SequentialGroup(
                    new ParallelDeadlineGroup(
                            new Delay(3),
                            new FollowPath(
                                    PedroComponent.follower().pathBuilder().addPath(
                                                    new BezierLine(
                                                            PedroComponent.follower().getPose(),
                                                            Poses.mirrorCoordinates(redShootingPose, RobotSubsystem.INSTANCE.getAllianceColor())
                                                    )
                                            )
                                            .setConstantHeadingInterpolation(
                                                    Poses.mirrorCoordinates(redShootingPose, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading())
                                            .build()
                            )
                    ),
                    RobotSubsystem.INSTANCE.Shoot(),
                    new FollowPath(
                            PedroComponent.follower().pathBuilder().addPath(
                                    new BezierLine(
                                            PedroComponent.follower().getPose(),
                                            Poses.mirrorCoordinates(redPark, RobotSubsystem.INSTANCE.getAllianceColor())
                                    )
                            ).build()
                    )
            ).schedule();
            killCycling = true;
        }

        if (autoTimer.getTimeSeconds() > 28.5 && !killCycling) {
            CommandManager.INSTANCE.cancelAll();
            new FollowPath(
                    PedroComponent.follower().pathBuilder().addPath(
                            new BezierLine(
                                    PedroComponent.follower().getPose(),
                                    Poses.mirrorCoordinates(redPark, RobotSubsystem.INSTANCE.getAllianceColor())
                            )
                    ).build()
            ).schedule();
            killCycling = true;
        }

        if (!killCycling) {
            autoPathUpdate();
        }
        telemetry.addData("Shooting delay: ", UniConstants.FAST_FLICKER_TIME_UP);
        telemetry.addData("Passed: ", passed);
//        telemetry.addData("Auto State: ", autoState);
        telemetry.addData("Cycling?: ", cycle);
        telemetry.addData("Stop cycling?: ", killCycling);
//        telemetry.addData("Follower X: ", PedroComponent.follower().getPose().getX());
//        telemetry.addData("Follower Y: ", PedroComponent.follower().getPose().getY());
//        telemetry.addData("Follower H: ", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
//        OuttakeSubsystem.INSTANCE.sendTelemetry();
    }

    @Override
    public void onStop() {
        UniConstants.FAST_FLICKER_TIME_UP = .3;
    }

    private void autoPathUpdate() {
        switch (autoState) {
            case -1:
                if (cycle) {
                    setAutoState(4);
                }
                break;
            case 1:
                if (oldState != autoState) {
                    new SequentialGroup(
                            new ParallelGroup(
                                    OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.LIME),
                                    OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.REACTIVE)
                                    //new InstantCommand(() -> OuttakeSubsystem.hoodLinreg = false),
//                                    new ParallelDeadlineGroup(
//                                            new Delay(1.25),
//                                            new FollowPath(startToShoot)
//
//                                    )
                            ),
                            SetPassed(true),
                            new Delay(.25),
                            RobotSubsystem.INSTANCE.Shoot(),
                            SetAutoState(2)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 2:
                if (oldState != autoState) {
                    new SequentialGroup(
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                            new ParallelDeadlineGroup(
                                    new Delay(2),
                                    new FollowPath(shootToFar)
                            ),
                            SetAutoState(3)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 3:
                if (oldState != autoState) {
                    new SequentialGroup(
                            new ParallelDeadlineGroup(
                                    new Delay(2.5),
                                    new FollowPath(farToShootLine)
                            ),
//                            new ParallelDeadlineGroup(
//                                    new Delay(.125),
//                                    new TurnTo(Angle.fromDeg(35))
//                            ),
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OUT),
                            new Delay(.075),
                            RobotSubsystem.INSTANCE.Shoot(),
                            SetAutoState(4)

                    ).schedule();
                }
                oldState = autoState;
                break;

            //Cycling
            case 4:
                if (oldState != autoState) {
                    new SequentialGroup(
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                            new ParallelDeadlineGroup(
                                    new Delay(1.25),
                                    new FollowPath(shootToCycle)
                            ),
                            SetAutoState(5)
                    ).schedule();
                }
                oldState = autoState;

                break;
            case 5:
                if (oldState != autoState) {
                    new SequentialGroup(
                            new ParallelDeadlineGroup(
                                    new Delay(2.75),
                                    new FollowPath(cycleToShoot)
                            ),
//                            new ParallelDeadlineGroup(
//                                    new Delay(.125),
//                                    new TurnTo(Angle.fromDeg(35))
//                            ),
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OUT),
                            new Delay(.075),
                            RobotSubsystem.INSTANCE.Shoot(),
                            SetAutoState(-1)
                    ).schedule();
                }
                oldState = autoState;
                break;

        }
    }

    private void setAutoState(int state) {
        if (state >= 0) {
            autoState = state;
        } else {
            autoState = -1;
        }
        pathTimer.reset();
    }

    public Command SetAutoState(int state) {
        return new InstantCommand(() -> setAutoState(state));
    }

    public Command SetPassed(boolean pass) {
        return new InstantCommand(() -> passed = pass);
    }

    public void createPaths() {
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
                .setBrakingStrength(10)
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
                .setBrakingStrength(10)
                .build();
        farToShootLine = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                Poses.mirrorCoordinates(redIntakeFarFinal, RobotSubsystem.INSTANCE.getAllianceColor()),
                                Poses.mirrorCoordinates(redShootingPose, RobotSubsystem.INSTANCE.getAllianceColor())
                        )
                )
                .setConstantHeadingInterpolation(
                        Poses.mirrorCoordinates(new Pose(0, 0, Math.toRadians(0)), RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
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
                        Poses.mirrorCoordinates(new Pose(0, 0, Math.toRadians(37.5)), RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
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
                        Poses.mirrorCoordinates(new Pose(0, 0, Math.toRadians(0)), RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                )
                .build();


    }

}