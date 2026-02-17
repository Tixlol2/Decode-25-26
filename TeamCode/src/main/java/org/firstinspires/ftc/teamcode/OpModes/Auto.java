package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
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

    private int autoState = 0;
    private int maxState = 0;
    private final Timer pathTimer = new Timer();

    private AutoSelect autoSelect = AutoSelect.SHORT;

    private Supplier<PathChain> parkClosePathSupplier;
    private Supplier<PathChain> shortShootPathSupplier;


    @Override
    public void onInit(){
        parkClosePathSupplier = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueClosePark : Poses.redClosePark)))
                .build();

        shortShootPathSupplier = () -> PedroComponent.follower()
                .pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower().getPose(), RobotSubsystem.INSTANCE.getAllianceColor() == RobotSubsystem.AllianceColor.BLUE ? Poses.blueAutoShoot : Poses.redAutoShoot)))
                .build();


    }

    @Override
    public void onWaitForStart(){



    }


    @Override
    public void onUpdate(){

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
                if(autoSelect.equals(AutoSelect.SHORT)) {
                    new SequentialGroup(
                            OuttakeSubsystem.INSTANCE.ScanPattern(),
                            OuttakeSubsystem.INSTANCE.SetFlywheelState(OuttakeSubsystem.FlywheelState.FULL),
                            new FollowPath(shortShootPathSupplier.get()),
                            RobotSubsystem.INSTANCE.Shoot()
                            ).schedule();
                }
                else if (autoSelect.equals(AutoSelect.FAR)){

                }


        }
    }

    private void selectAuto(){

    }

    enum AutoSelect{
        SHORT,
        FAR
    }


}
