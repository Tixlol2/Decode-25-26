package org.firstinspires.ftc.teamcode.OpModes.Autos;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.Util.AutoCommands;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

//12 Ball Close
@Autonomous(name = "Far Solo", group = "Main")
public class FarSolo extends NextFTCOpMode {

    {
        addComponents(
                new SubsystemComponent(RobotSubsystem.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }


    public static Pose prevPose = new Pose();

    boolean cycling = false;

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
    public void onStartButtonPressed() {
        OuttakeSubsystem.INSTANCE.resetTurret();
        RobotSubsystem.INSTANCE.resetPattern();
//        PedroComponent.follower().setStartingPose(Poses.mirrorCoordinates(Poses.blueFarStart, RobotSubsystem.INSTANCE.getAllianceColor()));

        follower().setStartingPose((RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueFarStart : Poses.redFarStart).withHeading(RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Math.toRadians(180) : Math.toRadians(0)));

        RobotSubsystem.inTele = false;
        RobotSubsystem.INSTANCE.updatingDist = true;
        autoState = 0;
    }

    @Override
    public void onUpdate() {

        Pose currentPose = PedroComponent.follower().getPose();
        if (!currentPose.roughlyEquals(new Pose(0, 0), 10)) {
            prevPose = PedroComponent.follower().getPose();

        }
        if (cycling && !CommandManager.INSTANCE.hasCommandsUsing("CYCLING")) {
            AutoCommands.cycle(AutoCommands.shootLocation.FAR, AutoCommands.cycleLocation.GATE, 3, 1).schedule();
        }
        if (ActiveOpMode.getRuntime() > 29 || (!cycling && autoState == -1)) {
            CommandManager.INSTANCE.cancelAll();
            AutoCommands.park(AutoCommands.shootLocation.FAR).schedule();
        }

        autoPathUpdate();
        telemetry.addData("Cycling: ", cycling);
        telemetry.addData("Cycling Test: ", !CommandManager.INSTANCE.hasCommandsUsing("CYCLING"));
        telemetry.addData("Auto State: ", autoState);
        OuttakeSubsystem.INSTANCE.sendTelemetry();
    }

    @Override
    public void onStop() {
        AutoCommands.prevPose = prevPose;
    }


    private void autoPathUpdate() {
        switch (autoState) {
            case -1:
                break;
            case 0:
                AutoCommands.init.schedule();
                setAutoState(1);
                break;
            case 1:
                if(oldState != autoState){
                    new SequentialGroup(
                            new ParallelRaceGroup(
                                    new Delay(5),
                                    AutoCommands.init,
                                    AutoCommands.shootPreload(AutoCommands.shootLocation.FAR, 2.5)
                            ),
                            SetAutoState(2)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 2:
                if (oldState != autoState) {
                    new SequentialGroup(
                            AutoCommands.farSpikeShoot(AutoCommands.shootLocation.FAR, 2.5, 1.5),
                            SetAutoState(3)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 3:
                if (oldState != autoState) {
                    new SequentialGroup(
                            AutoCommands.midSpikeShoot(AutoCommands.shootLocation.FAR, AutoCommands.pathType.LINE, false, 2.5, 1.5),
                            SetAutoState(4)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 4:
                if (oldState != autoState) {
                    new SequentialGroup(
                            AutoCommands.cycle(AutoCommands.shootLocation.FAR, AutoCommands.cycleLocation.GATE, 2.5, 1.5),
                            SetAutoState(5)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 5:
                if (oldState != autoState) {
                    new SequentialGroup(
                            AutoCommands.humanPlayerShoot(2.5, 1.5),
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
    }

    public Command SetAutoState(int state) {
        return new InstantCommand(() -> setAutoState(state));
    }


}