package org.firstinspires.ftc.teamcode.OpModes;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystemLL;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class NewAuto extends NextFTCOpMode {

    {
        addComponents(
                new SubsystemComponent(RobotSubsystem.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private final Pose redStartingPose = new Pose(127, 120, Math.toRadians(36));
    private final Pose blueStartingPose = new Pose(16, 120, Math.toRadians(144));

    private final Pose redShootingPose = new Pose(92, 84, Math.toRadians(45));
    private final Pose blueShootingPose = new Pose(54, 84, Math.toRadians(150));

    private final Pose redIntakeCloseFinal = new Pose(124, 83, 0);

    private final Pose redIntakeMidFinal = new Pose(128, 58, 0);
    private final Pose redIntakeMidCP = new Pose(78.63924050632912, 59.78481012658228);

    private final Pose redIntakeFarFinal = new Pose(128, 34, 0);
    private final Pose redIntakeFarCP = new Pose(74.30379746835442, 28.56962025316455);

    private final Pose redLeverPark = new Pose(120, 70);

    private final Pose redClosePark = new Pose(84, 134);

    private final Pose redLeverStrafeGoal = new Pose(160, 70);

    private PathChain startToShoot, shootToMid, midToShoot, shootToClose, closeToShoot, shootToParkLever, shootToParkClose, shootToFar, farToShootCurved, farToShootLine, hitLeverFromMid, backFromLever, backFromMid;

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
        UniConstants.FAST_FLICKER_TIME_UP = .35;
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
            case 2:
                if(oldState != autoState){
                    new SequentialGroup(
//                            new Delay(5),
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                            new ParallelDeadlineGroup(
                                    new Delay(2.25),
                                    new FollowPath(shootToMid)
                            ),
//                            new TurnBy(Angle.fromDeg(-5)),
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OFF),
                            new ParallelDeadlineGroup(
                                    new Delay(.25),
                                    new FollowPath(backFromMid)
                            ),
                            new ParallelDeadlineGroup(
                                    new Delay(1.5),
                                    new FollowPath(hitLeverFromMid)
                            ),
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                            new ParallelDeadlineGroup(
                                    new Delay(.125),
                                    new FollowPath(backFromLever)
                                    ),
                            SetAutoState(3)
                    ).schedule();
                    oldState = autoState;
                }
                break;
            case 3:
                if(oldState != autoState){
                    new SequentialGroup(
//                            new Delay(5),
                            new ParallelGroup(
                                    OuttakeSubsystem.INSTANCE.ScanPattern(),
                                    new FollowPath(midToShoot)
                            ),
                            new ParallelGroup(
                                    OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.GOAL)
//                            new ParallelDeadlineGroup(
//                                    new Delay(.125),
//                                    new TurnTo(Angle.fromDeg(42))
//                            )),
                            ),
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OUT),
//                            OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.GOAL),
                            new Delay(.3),
                            RobotSubsystem.INSTANCE.AutoShoot(),
                            SetAutoState(4)
                    ).schedule();
                    oldState = autoState;
                }
                break;
            case 4:
                if(oldState != autoState){
                    new SequentialGroup(
//                            new Delay(5),
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                            new ParallelDeadlineGroup(
                                    new Delay(1.25),
                                    new FollowPath(shootToClose)
                            ),
                            SetAutoState(5)
                    ).schedule();
                    oldState = autoState;
                }
                break;
            case 5:
                if(oldState != autoState){
                    new SequentialGroup(
//                            new Delay(5),
                            new ParallelDeadlineGroup(
                                    new Delay(1.25),
                                    new ParallelGroup(
                                            new FollowPath(closeToShoot),
                                            OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.GOAL)
                                    )
                            ),
//                            new ParallelDeadlineGroup(
//                                    new Delay(.125),
//                                    new TurnTo(Angle.fromDeg(43))
//                            ),
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OUT),
                            new Delay(.3),
                            RobotSubsystem.INSTANCE.AutoShoot(),
                            SetAutoState(6)
                    ).schedule();
                    oldState = autoState;
                }
                break;
            case 6:
                if(oldState != autoState){
                    new SequentialGroup(
//                            new Delay(5),
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                            new ParallelDeadlineGroup(
                                    new Delay(3.5),
                                    new FollowPath(shootToFar)
                            ),
                            SetAutoState(7)
                    ).schedule();
                    oldState = autoState;
                }
                break;
            case 7:
                if(oldState != autoState){
                    new SequentialGroup(
//                            new Delay(5),
                            new ParallelDeadlineGroup(
                                    new Delay(3),
                                    new FollowPath(farToShootLine)
                            ),
//                            new ParallelDeadlineGroup(
//                                    new Delay(.125),
//                                    new TurnTo(Angle.fromDeg(35))
//                            ),
                            IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OUT),
                            new Delay(.3),
                            RobotSubsystem.INSTANCE.AutoShoot(),
                            SetAutoState(8)
                    ).schedule();
                    oldState = autoState;
                }
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
                .addPath(new BezierLine(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redStartingPose : blueStartingPose,
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redShootingPose : blueShootingPose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(37))
                .build();
        shootToMid = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redShootingPose : blueShootingPose,
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeMidCP : blueShootingPose, //TODO: Update
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeMidFinal : blueShootingPose //TODO: Update

                ))
                .setConstantHeadingInterpolation(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeMidFinal.getHeading() : blueStartingPose.getHeading() //Todo: update
                )
                .build();
        midToShoot = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeMidFinal : blueShootingPose,
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeMidCP : blueShootingPose, //TODO: Update
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redShootingPose : blueShootingPose //TODO: Update

                ))
                .setLinearHeadingInterpolation(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeMidFinal.getHeading() : blueShootingPose.getHeading(),
                        Math.toRadians(90) //Todo: update
                )
                .build();
        shootToClose = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redShootingPose : blueShootingPose,
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeCloseFinal : blueShootingPose
                ))
                .setConstantHeadingInterpolation(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeMidFinal.getHeading() : blueStartingPose.getHeading() //Todo: update
                )
                .build();
        closeToShoot = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeCloseFinal : blueStartingPose,
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redShootingPose.plus(new Pose(5, 5)) : blueShootingPose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
        shootToFar = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redShootingPose : blueShootingPose,
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeFarCP : redIntakeFarFinal, //TODO update
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeFarFinal : redIntakeFarCP //Todo update
                ))
                .setConstantHeadingInterpolation(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeFarFinal.getHeading() : Math.toRadians(180))
                .build();
        farToShootCurved = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeFarFinal : blueShootingPose,
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeFarCP : redIntakeFarFinal, //TODO update
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redShootingPose : blueShootingPose //Todo update
                ))
                .setTangentHeadingInterpolation()
                .build();
        farToShootLine = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeFarFinal : blueShootingPose,//TODO update
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redShootingPose.plus(new Pose(6.7, 30)) : blueShootingPose //Todo update
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        shootToParkClose = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redShootingPose : blueShootingPose,
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redClosePark : blueShootingPose
                ))
                .build();
        shootToParkLever = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redShootingPose : blueShootingPose,
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redLeverPark : blueShootingPose
                ))
                .build();
        hitLeverFromMid = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeMidFinal : blueShootingPose,
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? new Pose(85, 58) : new Pose(),
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redLeverStrafeGoal : redLeverPark
                        ))
                .setBrakingStart(4)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        backFromLever = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(
                        RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redLeverStrafeGoal : redLeverPark,
                        redLeverStrafeGoal.minus(new Pose(30, 0))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
        backFromMid = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeMidFinal : blueShootingPose,
                                RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.RED ? redIntakeMidFinal.minus(new Pose(20, 0)) : blueShootingPose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


    }

}
