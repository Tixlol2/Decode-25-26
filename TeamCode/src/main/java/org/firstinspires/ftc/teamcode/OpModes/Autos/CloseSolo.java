package org.firstinspires.ftc.teamcode.OpModes.Autos;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.Util.AutoCommands;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

//12 Ball Close
@Autonomous(name = "Close Solo", group = "Main")
public class CloseSolo extends NextFTCOpMode {

    {
        addComponents(
                new SubsystemComponent(RobotSubsystem.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }



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
        AutoCommands.startButton(AutoCommands.shootLocation.CLOSE).schedule();
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
        AutoCommands.prevPose = prevPose;
    }


    private void autoPathUpdate(){
        switch (autoState){
            case -1:
                break;
            case 0:
                setAutoState(1);
                break;
            case 1:
                if(oldState != autoState){
                    new SequentialGroup(
                            new ParallelGroup(
                                    AutoCommands.init,
                                    AutoCommands.shootPreload(AutoCommands.shootLocation.CLOSE, 2.5)
                            ),
                            SetAutoState(2)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 2:
                if(oldState != autoState){
                    new SequentialGroup(
                            AutoCommands.closeSpikeShoot(AutoCommands.shootLocation.CLOSE, true,2.5, 1.5),
                            SetAutoState(3)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 3:
                if(oldState != autoState){
                    new SequentialGroup(
                            AutoCommands.midSpikeShoot(AutoCommands.shootLocation.CLOSE, AutoCommands.pathType.LINE, false,2.5, 1.5),
                            SetAutoState(4)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 4:
                if(oldState != autoState){
                    new SequentialGroup(
                            AutoCommands.farSpikeShoot(AutoCommands.shootLocation.CLOSE, 4, 1.5),
                            SetAutoState(5)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 5:
                if(oldState != autoState){
                    new SequentialGroup(
                            AutoCommands.park(AutoCommands.shootLocation.CLOSE),
                            SetAutoState(-1)
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




}