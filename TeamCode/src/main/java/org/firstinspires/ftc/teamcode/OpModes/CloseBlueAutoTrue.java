package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.BetterVisionTM;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSortingSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Close Blue True", group = "Main")
public class CloseBlueAutoTrue extends NextFTCOpMode {
    {
        addComponents(
                new SubsystemComponent(TurretSubsystem.INSTANCE, IntakeSortingSubsystem.INSTANCE,  BetterVisionTM.INSTANCE, MecDriveSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    CloseBluePaths paths;
    Follower follower;

    JoinedTelemetry joinedTelemetry;

    int pathState = 0;
    Timer pathTimer = new Timer();



    boolean turretForward = false;
    boolean aimAtGoal = true;

    @Override
    public void onInit(){
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        follower = MecDriveSubsystem.INSTANCE.getFollower();
        follower.setStartingPose(Poses.blueTopStart);
        paths = new CloseBluePaths(follower);
        NextFTCTeleop.patternFull = false;
    }

    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            NextFTCTeleop.color = UniConstants.teamColor.RED;
        } else if (gamepad1.b) {
            NextFTCTeleop.color = UniConstants.teamColor.BLUE;
        }

        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("Circle for Blue, X for Red ");
        joinedTelemetry.addData("Current Team Color ", NextFTCTeleop.color);
        joinedTelemetry.update();
    }

    @Override
    public void onStartButtonPressed(){
        new SequentialGroup(
                //Path 1: Shoot close preload
                new SequentialGroup(
                        TurretSubsystem.INSTANCE.SetMotorPower(.67),
                        AimForward(),
                        new ParallelGroup(
                            new Delay(2.5),
                            MecDriveSubsystem.INSTANCE.FollowPath(paths.Path1, true)
                                ),
                        IntakeSortingSubsystem.INSTANCE.shoot(NextFTCTeleop.pattern)
                        ),
                ChangePath(),
                //Path 2: Prepare to intake
                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path2, false),
                ChangePath(),
                //Path 3: Intake and hit lever
                new SequentialGroup(
                        IntakeSortingSubsystem.INSTANCE.runActive(),
                        MecDriveSubsystem.INSTANCE.FollowPathTime(paths.Path3, true, .85, 2),
                        AimObelisk()
                ),
                ChangePath(),

                //Path 4: Shoot
                new SequentialGroup(
                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path4, true),
                        new Delay(.5),
                        AimGoal(),
                        new Delay(.25),
                        IntakeSortingSubsystem.INSTANCE.stopActive(),
                        IntakeSortingSubsystem.INSTANCE.shoot(NextFTCTeleop.pattern)
                        ),
                //Path 5: Prepare to intake
                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path5, false),
                AimForward(),
                ChangePath(),
                //Path 6: Intake
                new ParallelGroup(
                        IntakeSortingSubsystem.INSTANCE.runActive(),
                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path6, false, .925)
                ),
                ChangePath(),
                //Path 7: Shoot
                new SequentialGroup(
                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path7, true),
                        new Delay(.25),
                        IntakeSortingSubsystem.INSTANCE.stopActive(),
                        IntakeSortingSubsystem.INSTANCE.shoot(NextFTCTeleop.pattern)
                ),
                //Path 8: Prepare to intake
                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path8, false),
                ChangePath(),
                //Path 9: Intake
                new ParallelGroup(
                        IntakeSortingSubsystem.INSTANCE.runActive(),
                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path9, false, .925)
                ),
                ChangePath(),
                //Path 10: Shoot
                new SequentialGroup(
                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path10, true),
                        new Delay(.25),
                        IntakeSortingSubsystem.INSTANCE.stopActive(),
                        IntakeSortingSubsystem.INSTANCE.shoot(NextFTCTeleop.pattern)
                ),
                //Path 11: Park
                new ParallelGroup(
                        StopSubsystems(),
                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path10, true)
                )
                ).schedule();
    }

    @Override
    public void onUpdate(){



        TurretSubsystem.INSTANCE.setTargetAngle(turretForward ? 0 : (aimAtGoal ? -MecDriveSubsystem.INSTANCE.getCalculatedTurretAngle() : -MecDriveSubsystem.INSTANCE.getObeliskAngle()));

        if(NextFTCTeleop.pattern.contains(null)){
            NextFTCTeleop.pattern = BetterVisionTM.INSTANCE.getPattern();
            if(!NextFTCTeleop.pattern.contains(null)){NextFTCTeleop.patternFull = true;}
        }

        joinedTelemetry.addData("Pattern: ", NextFTCTeleop.pattern);
        joinedTelemetry.addData("Pattern Full: ", NextFTCTeleop.patternFull);
        joinedTelemetry.addData("Path State: ", pathState);
        //joinedTelemetry.addData("Current Commands: ", CommandManager.INSTANCE.snapshot());
        MecDriveSubsystem.INSTANCE.sendTelemetry(UniConstants.loggingState.ENABLED);

        joinedTelemetry.update();
        follower.update();

    }


    public Command ChangePath(){
        return new InstantCommand(() -> {
            pathState++;
            pathTimer.reset();
        });
    }

    public Command ResetTimer(){
        return new InstantCommand(() -> pathTimer.reset());
    }

    public Command StopSubsystems(){
        return new ParallelGroup(
                IntakeSortingSubsystem.INSTANCE.stopActive(),
                TurretSubsystem.INSTANCE.SetMotorPower(0),
                new InstantCommand(() -> turretForward = false)
        );
    }

    public Command AimForward(){
        return new InstantCommand(() -> {turretForward = true; aimAtGoal = true;});
    }

    public Command AimGoal(){
        return new InstantCommand(() -> {turretForward = false; aimAtGoal = true;});
    }

    public Command AimObelisk(){
        return new InstantCommand(() -> {turretForward = false; aimAtGoal = false;});
    }


}
