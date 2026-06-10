package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;

@Configurable
public class AutoCommands {

    public static Pose blueFarSpike = new Pose(13.3718411552, 38.653429602888075, Math.toRadians(180));
    public static Pose blueFarSpikeCP = new Pose(66.28720924918886, 30.7152584198);

    public static Pose blueCloseShooting = new Pose(47.22743682310469, 97.2454873646, Math.toRadians(90));

    public static Pose blueFarShooting = new Pose(56.346570397111904, 8.173285198555952, Math.toRadians(90));

    public static Pose blueMiddleSpike = new Pose(10.3971119134, 63.2599277978, Math.toRadians(180));
    public static Pose blueMiddleSpikeCP1 = new Pose(70.03399899465339, 59.47806516473976);

    public static Pose blueCloseSpike = new Pose(20.0902527076, 87.17328519855594, Math.toRadians(180));
    public static Pose blueCloseSpikeCP1 = new Pose(57.94438605310059, 85.41863547045658);

    public static Pose blueCloseCycle = new Pose(13.718411552346563, 59.91335740072203, Math.toRadians(160));
    public static Pose blueCloseCycleCP = new Pose(39.75090252707583, 52.85198555956682);

    //Far cycle doesnt need a CP if we are only running into HP and taking any in there
    public static Pose blueHumanPlayer = new Pose(13.718411552346563, 59.91335740072203, Math.toRadians(180));

    public static Pose blueFarCycle = new Pose(13.025270758122742, 42.93140794223829, Math.toRadians(180));
    public static Pose blueFarCycleCP1 = new Pose(1.906137184115518, 5.346570397111908);
    public static Pose blueFarCycleCP2 = new Pose(16.476534296028884, 27.646209386281587);

    public static Pose blueClosePark = new Pose(56.01444043321301, 132.59205776173278);
    public static Pose blueFarPark = new Pose(15.451263537906144, 11.465703971119035);

    public static Pose blueGateBumpHigh = new Pose(17.0, 79.5, Math.toRadians(180));
    public static Pose blueGateBumpCPHigh = new Pose(80, 77.38086642599279);

    public static Pose blueGateBumpLow = new Pose(17.42599277978339, 67.97472924187724, Math.toRadians(180));
    public static Pose blueGateBumpCPLow = new Pose(44.732851985559556, 68.543321299639);

    public static double shootDelay = .35;


    public static Command farSpikeShoot(shootLocation loc, double intakeTime, double returnTime) {

        return new SequentialGroup(

                //From current pos, go to end of spike mark
                intakeFar(intakeTime),

                //Go to shooting pos based on input
                shoot(loc, returnTime)
        );

    }

    public static Command midSpikeShoot(shootLocation loc, pathType type, boolean bumpGate, double intakeTime, double returnTime) {

        return new SequentialGroup(

                //From current pos, go to end of spike mark
                midIntake(intakeTime),
                new IfElseCommand(() -> bumpGate,
                        gateBump(1.5, false)
                ),
                //Go to shooting pos based on input
                shoot(loc, returnTime)
        );

    }

    public static Command closeSpikeShoot(shootLocation loc, boolean bumpGate, double intakeTime, double returnTime) {

        return new SequentialGroup(

                //From current pos, go to end of spike mark
                intakeClose(intakeTime),
                new IfElseCommand(() -> bumpGate,
                        gateBump(1, true)
                ),

                //Go to shooting pos based on input
                shoot(loc, returnTime)
        );

    }

    public static Command cycle(shootLocation loc, cycleLocation cycle, double intakeTime, double returnTime) {

        return new SequentialGroup(

                //From current pos, go to gate/human player
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                new IfElseCommand(() -> cycle == cycleLocation.GATE,
                        new ParallelRaceGroup(
                                new Delay(intakeTime),
                                new FollowPath(
                                        PedroComponent.follower().pathBuilder()
                                                .addPath(
                                                        new BezierCurve(
                                                                PedroComponent.follower().getPose(),
                                                                Poses.mirrorCoordinates(blueCloseCycleCP, RobotSubsystem.INSTANCE.getAllianceColor()),
                                                                Poses.mirrorCoordinates(blueCloseCycle, RobotSubsystem.INSTANCE.getAllianceColor())

                                                        )
                                                )
                                                .setConstantHeadingInterpolation(
                                                        Poses.mirrorCoordinates(blueCloseCycle, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                                                ).build()
                                )),
                        new ParallelRaceGroup(
                                new Delay(intakeTime),
                                new FollowPath(
                                        PedroComponent.follower().pathBuilder()
                                                .addPath(
                                                        new BezierCurve(
                                                                PedroComponent.follower().getPose(),
                                                                Poses.mirrorCoordinates(blueFarCycleCP1, RobotSubsystem.INSTANCE.getAllianceColor()),
                                                                Poses.mirrorCoordinates(blueFarCycleCP2, RobotSubsystem.INSTANCE.getAllianceColor()),
                                                                Poses.mirrorCoordinates(blueFarCycle, RobotSubsystem.INSTANCE.getAllianceColor())
                                                        )
                                                )
                                                .setConstantHeadingInterpolation(
                                                        Poses.mirrorCoordinates(blueFarShooting, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                                                ).build()
                                ))

                ),

                //Go to shooting pos based on input
                new ParallelRaceGroup(
                        new Delay(returnTime),
                        new FollowPath(
                                PedroComponent.follower().pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        PedroComponent.follower().getPose(),
                                                        loc == shootLocation.CLOSE ?
                                                                Poses.mirrorCoordinates(blueCloseShooting, RobotSubsystem.INSTANCE.getAllianceColor()) :
                                                                Poses.mirrorCoordinates(blueFarShooting, RobotSubsystem.INSTANCE.getAllianceColor()))
                                        )
                                        .setConstantHeadingInterpolation(
                                                Poses.mirrorCoordinates(blueFarShooting, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                                        ).build()
                        )),
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OUT),
                new Delay(shootDelay),
                RobotSubsystem.INSTANCE.AutoShoot(),
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OFF)
        ).addRequirements("CYCLING");

    }

    public static Command shootPreload(shootLocation loc, double time) {
        return new SequentialGroup(
                new IfElseCommand(() -> loc == shootLocation.CLOSE,
                        new ParallelRaceGroup(
                                new Delay(time),
                                new FollowPath(
                                        PedroComponent.follower().pathBuilder()
                                                .addPath(
                                                        new BezierLine(
                                                                Poses.mirrorCoordinates(Poses.blueCloseStart, RobotSubsystem.INSTANCE.getAllianceColor()),
                                                                Poses.mirrorCoordinates(blueCloseShooting, RobotSubsystem.INSTANCE.getAllianceColor())
                                                        )
                                                )
                                                .setConstantHeadingInterpolation(
                                                        Poses.mirrorCoordinates(blueCloseShooting, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                                                ).build()
                                )
                        )
                ),
                new Delay(shootDelay),
                RobotSubsystem.INSTANCE.AutoShoot()
        );
    }

    public static Command park(shootLocation loc) {
        return new FollowPath(
                PedroComponent.follower().pathBuilder()
                        .addPath(
                                new BezierLine(
                                        PedroComponent.follower().getPose(),
                                        loc == shootLocation.CLOSE ?
                                                Poses.mirrorCoordinates(blueClosePark, RobotSubsystem.INSTANCE.getAllianceColor()) :
                                                Poses.mirrorCoordinates(blueFarPark, RobotSubsystem.INSTANCE.getAllianceColor())
                                )
                        )
                        .setTangentHeadingInterpolation().build()
        );
    }

    public static Command parkShoot(shootLocation loc, double timer) {
        return new SequentialGroup(
                new ParallelRaceGroup(
                        new Delay(timer),
                        new FollowPath(
                                PedroComponent.follower().pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        PedroComponent.follower().getPose(),
                                                        loc == shootLocation.CLOSE ?
                                                                Poses.mirrorCoordinates(blueClosePark, RobotSubsystem.INSTANCE.getAllianceColor()) :
                                                                Poses.mirrorCoordinates(blueFarPark, RobotSubsystem.INSTANCE.getAllianceColor())
                                                )
                                        )
                                        .setLinearHeadingInterpolation(PedroComponent.follower().getPose().getHeading(), Poses.mirrorCoordinates(blueCloseSpike, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()).build()
                        )),
                RobotSubsystem.INSTANCE.AutoShoot()
        );
    }

    public static Command gateBump(double timer, boolean high) {
        return new ParallelRaceGroup(
                new Delay(timer),
                new FollowPath(
                        PedroComponent.follower().pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                PedroComponent.follower().getPose(),
                                                Poses.mirrorCoordinates(high ? blueGateBumpHigh : blueGateBumpLow, RobotSubsystem.INSTANCE.getAllianceColor())
                                        )
                                )
                                .setConstantHeadingInterpolation(Poses.mirrorCoordinates(new Pose(0,0, Math.toRadians(180)), RobotSubsystem.INSTANCE.getAllianceColor()).getHeading())
                                .build()
                )
        );
    }

    public static Command humanPlayerShoot(double intake, double returnTime) {
        return new SequentialGroup(
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                new ParallelRaceGroup(
                        new Delay(intake),
                        new FollowPath(
                                PedroComponent.follower().pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        PedroComponent.follower().getPose(),
                                                        Poses.mirrorCoordinates(blueHumanPlayer, RobotSubsystem.INSTANCE.getAllianceColor())
                                                )
                                        )
                                        .setConstantHeadingInterpolation(Poses.mirrorCoordinates(blueHumanPlayer, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading())
                                        .build()
                        )),

                new ParallelRaceGroup(
                        new Delay(returnTime),
                        new FollowPath(
                                PedroComponent.follower().pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        PedroComponent.follower().getPose(),
                                                        Poses.mirrorCoordinates(blueFarShooting, RobotSubsystem.INSTANCE.getAllianceColor())
                                                )
                                        ).setConstantHeadingInterpolation(Poses.mirrorCoordinates(blueFarShooting, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()).build()

                        )),
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OUT),
                new Delay(shootDelay),
                RobotSubsystem.INSTANCE.AutoShoot(),
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OFF)
        );

    }

    public static Command init =
            new ParallelGroup(
                    OuttakeSubsystem.INSTANCE.SetTurretState(OuttakeSubsystem.TurretState.LIME),
                    OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.REACTIVE)
            );

    public static Command startButton(shootLocation loc) {
        return new InstantCommand(() -> {
            OuttakeSubsystem.INSTANCE.resetTurret();
            RobotSubsystem.INSTANCE.resetPattern();
            PedroComponent.follower().setStartingPose(Poses.mirrorCoordinates(loc == shootLocation.CLOSE ? Poses.blueCloseStart : Poses.blueFarStart, RobotSubsystem.INSTANCE.getAllianceColor()));
            RobotSubsystem.inTele = false;
            RobotSubsystem.INSTANCE.updatingDist = true;
        });
    }

    public static void setRPM(double rpm) {
        OuttakeSubsystem.setTurretState(OuttakeSubsystem.TurretState.GOAL);
        OuttakeSubsystem.INSTANCE.setLauncherState(OuttakeSubsystem.FlywheelState.LAZY);
        OuttakeSubsystem.lazyRPM = rpm;
    }

    public static Command intakeFar(double intakeTime){
        return new SequentialGroup(
        IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                new ParallelRaceGroup(
                        new Delay(intakeTime),
                        new FollowPath(PedroComponent.follower().pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                Poses.mirrorCoordinates(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor()),
                                                Poses.mirrorCoordinates(blueFarSpikeCP, RobotSubsystem.INSTANCE.getAllianceColor()),
                                                Poses.mirrorCoordinates(blueFarSpike, RobotSubsystem.INSTANCE.getAllianceColor())
                                        )
                                )
                                .setConstantHeadingInterpolation(
                                        Poses.mirrorCoordinates(blueFarSpike, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                                ).build())));
    }

    public static Command intakeClose(double intakeTime){
        return new SequentialGroup(

                //From current pos, go to end of spike mark
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                new ParallelRaceGroup(
                        new Delay(intakeTime),
                        new FollowPath(
                                PedroComponent.follower().pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        PedroComponent.follower().getPose(),
                                                        Poses.mirrorCoordinates(blueCloseSpikeCP1, RobotSubsystem.INSTANCE.getAllianceColor()),
//                                                        Poses.mirrorCoordinates(blueCloseSpikeCP2, RobotSubsystem.INSTANCE.getAllianceColor()),
                                                        Poses.mirrorCoordinates(blueCloseSpike, RobotSubsystem.INSTANCE.getAllianceColor())
                                                )
                                        )
                                        .setConstantHeadingInterpolation(
                                                Poses.mirrorCoordinates(blueCloseSpike, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                                        ).build()
                        )),
        new ParallelRaceGroup(
                new Delay(.35),
                new FollowPath(
                        PedroComponent.follower().pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                PedroComponent.follower().getPose(),
                                                Poses.mirrorCoordinates(blueCloseShooting, RobotSubsystem.INSTANCE.getAllianceColor())
                                        )
                                )
                                .setConstantHeadingInterpolation(
                                        Poses.mirrorCoordinates(blueCloseSpike, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                                ).build()))
        );
    }

    public static Command shoot(shootLocation loca, double timer){
//        PedroComponent.follower().breakFollowing();
        return new SequentialGroup(
                new ParallelRaceGroup(
                new Delay(timer),
                new FollowPath(
                        PedroComponent.follower().pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                PedroComponent.follower().getPose(),
                                                loca == shootLocation.CLOSE ?
                                                        Poses.mirrorCoordinates(blueCloseShooting, RobotSubsystem.INSTANCE.getAllianceColor()) :
                                                        loca == shootLocation.PARK ?
                                                                Poses.mirrorCoordinates(blueClosePark, RobotSubsystem.INSTANCE.getAllianceColor()) :
                                                                Poses.mirrorCoordinates(blueFarShooting, RobotSubsystem.INSTANCE.getAllianceColor()))
                                )
                                .setConstantHeadingInterpolation(
                                        PedroComponent.follower().getPose().getHeading()
                                ).build()

                )),
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OUT),
                new Delay(shootDelay),
                RobotSubsystem.INSTANCE.AutoShoot(),
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.OFF)
        );
    }

    public static Command midIntake(double intakeTime){
        return new SequentialGroup(

                //From current pos, go to end of spike mark
                IntakeSubsystem.INSTANCE.setActiveStateCommand(IntakeSubsystem.IntakeState.IN),
                new ParallelRaceGroup(
                        new Delay(intakeTime),
                        new FollowPath(
                                PedroComponent.follower().pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        PedroComponent.follower().getPose(),
                                                        Poses.mirrorCoordinates(blueMiddleSpikeCP1, RobotSubsystem.INSTANCE.getAllianceColor()),
//                                                        Poses.mirrorCoordinates(blueMiddleSpikeCP2, RobotSubsystem.INSTANCE.getAllianceColor()),
                                                        Poses.mirrorCoordinates(blueMiddleSpike, RobotSubsystem.INSTANCE.getAllianceColor())
                                                )
                                        )
                                        .setConstantHeadingInterpolation(
                                                Poses.mirrorCoordinates(blueMiddleSpike, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                                        ).build()
                        )
                ),
                new ParallelRaceGroup(
                        new Delay(.5),
                        new FollowPath(
                                PedroComponent.follower().pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        PedroComponent.follower().getPose(),
                                                        Poses.mirrorCoordinates(new Pose(22.3971119134, 63.2599277978), RobotSubsystem.INSTANCE.getAllianceColor())
                                                )
                                        )
                                        .setConstantHeadingInterpolation(
                                                Poses.mirrorCoordinates(blueMiddleSpike, RobotSubsystem.INSTANCE.getAllianceColor()).getHeading()
                                        ).build()
                )));
    }

    public enum shootLocation {
        CLOSE,
        FAR,
        PARK
    }

    public enum cycleLocation {
        GATE,
        HP
    }

    public enum pathType {
        CURVE,
        LINE
    }

    public static Pose prevPose = new Pose();

}
