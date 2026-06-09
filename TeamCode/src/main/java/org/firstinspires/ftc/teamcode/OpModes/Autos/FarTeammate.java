package org.firstinspires.ftc.teamcode.OpModes.Autos;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.Util.AutoCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

//12 Ball Close
@Autonomous(name = "Far Teammate", group = "Main")
public class FarTeammate extends NextFTCOpMode {

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
    public void onStartButtonPressed(){
        AutoCommands.startButton(AutoCommands.shootLocation.FAR).schedule();
        autoState = 1;
        OuttakeSubsystem.INSTANCE.SetTurretState( OuttakeSubsystem.TurretState.LIME);
    }

    @Override
    public void onUpdate(){

        Pose currentPose = PedroComponent.follower().getPose();
        if(!currentPose.roughlyEquals(new Pose(0, 0), 10)){
            prevPose = PedroComponent.follower().getPose();

        }
        if(cycling && !CommandManager.INSTANCE.hasCommandsUsing("CYCLING")){
            AutoCommands.cycle(AutoCommands.shootLocation.FAR, AutoCommands.cycleLocation.HP).schedule();
        }
        if(ActiveOpMode.getRuntime() > 29){
            CommandManager.INSTANCE.cancelAll();
            AutoCommands.park(AutoCommands.shootLocation.FAR).schedule();
        }

        autoPathUpdate();
        telemetry.addData("Cycling: ", cycling);
        telemetry.addData("Cycling Test: ", !CommandManager.INSTANCE.hasCommandsUsing("CYCLING"));
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
                AutoCommands.init.schedule();
                setAutoState(1);
                break;
            case 1:
                if(oldState != autoState){
                    new SequentialGroup(
                            AutoCommands.shootPreload(AutoCommands.shootLocation.FAR, 0),
                            SetAutoState(2)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 2:
                if(oldState != autoState){
                    new SequentialGroup(
                            AutoCommands.humanPlayerShoot(),
                            SetAutoState(3)
                    ).schedule();
                }
                oldState = autoState;
                break;
            case 3:
                if(oldState != autoState){
                    cycling = true;
                    setAutoState(-1);
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
    }

    public Command SetAutoState(int state){
        return new InstantCommand(() -> setAutoState(state));
    }




}