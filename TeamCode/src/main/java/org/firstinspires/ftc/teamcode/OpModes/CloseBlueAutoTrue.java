package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.AutonUtil.CloseBluePaths;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.BetterVisionTM;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSortingSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Disabled
@Autonomous(name = "12 Ball Blue Close", group = "Main")
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
        follower.setStartingPose(Poses.blueGoalTopStartFacing);
        paths = new CloseBluePaths(follower);
        TurretSubsystem.INSTANCE.setMotorPower(0);
    }

    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            Robot.color = UniConstants.teamColor.RED;
        } else if (gamepad1.b) {
            Robot.color = UniConstants.teamColor.BLUE;
        }

        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("Circle for Blue, X for Red ");
        joinedTelemetry.addData("Current Team Color ", Robot.color);
        joinedTelemetry.update();
    }

//    @Override
//    public void onStartButtonPressed(){
//        new SequentialGroup(
//                //Path 1: Shoot close preload
//                new SequentialGroup(
//                        TurretSubsystem.INSTANCE.SetMotorPower(.7),
//                        AimForward(),
//                        new ParallelGroup(
//                            new Delay(3),
//                            MecDriveSubsystem.INSTANCE.FollowPath(paths.Path1, true)
//                                ),
//                        new ParallelGroup(
//                        IntakeSortingSubsystem.INSTANCE.shoot(NextFTCTeleop.pattern),
//                                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path12, true, .125)
//                        )),
//                ChangePath(),
//                //Path 2: Prepare to intake
//                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path2, false),
//                ChangePath(),
//                //Path 3: Intake and hit lever
//                new SequentialGroup(
//                        IntakeSortingSubsystem.INSTANCE.runActive(),
//                        MecDriveSubsystem.INSTANCE.FollowPathTime(paths.Path3, true, .65, 2),
//                        AimObelisk()
//                ),
//                ChangePath(),
//
//                //Path 4: Shoot
//                new SequentialGroup(
//                        AimGoal(),
//                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path4, true),
//                        new Delay(.5),
//
//                                IntakeSortingSubsystem.INSTANCE.shoot(NextFTCTeleop.pattern),
//                                //MecDriveSubsystem.INSTANCE.FollowPath(paths.Path13, false, .125)
//
//                        IntakeSortingSubsystem.INSTANCE.stopActive()
//                        ),
//                //Path 5: Prepare to intake
//                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path5, false),
//                //AimForward(),
//                ChangePath(),
//                //Path 6: Intake
//                new ParallelGroup(
//                        IntakeSortingSubsystem.INSTANCE.runActive(),
//                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path6, false)
//                ),
//                ChangePath(),
//                //Path 7: Shoot
//                new SequentialGroup(
//                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path7, true),
//                        new Delay(.25),
//                        IntakeSortingSubsystem.INSTANCE.stopActive(),
//                        new ParallelGroup(
//                                IntakeSortingSubsystem.INSTANCE.shoot(NextFTCTeleop.pattern),
//                                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path12, true, .125)
//                        )
//                ),
//                //Path 8: Prepare to intake
//                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path8, false),
//                ChangePath(),
//                //Path 9: Intake
//                new ParallelGroup(
//                        IntakeSortingSubsystem.INSTANCE.runActive(),
//                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path9, false, .8)
//                ),
//                ChangePath(),
//                //Path 10: Shoot
//                new SequentialGroup(
//                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path10, true),
//                        new Delay(.25),
//                        IntakeSortingSubsystem.INSTANCE.stopActive(),
//                        new ParallelGroup(
//                                IntakeSortingSubsystem.INSTANCE.shoot(NextFTCTeleop.pattern),
//                                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path12, true, .125)
//                        )
//                ),
//                //Path 11: Park
//                new ParallelGroup(
//                        StopSubsystems(),
//                        MecDriveSubsystem.INSTANCE.FollowPath(paths.Path10, true)
//                )
//                ).schedule();
//    }

    @Override
    public void onUpdate(){



//        TurretSubsystem.INSTANCE.setTargetAngle(turretForward ? 0 : (aimAtGoal ? -MecDriveSubsystem.INSTANCE.getCalculatedTurretAngle() : -MecDriveSubsystem.INSTANCE.getObeliskAngle()));




        joinedTelemetry.addData("Path State: ", pathState);
        //joinedTelemetry.addData("Current Commands: ", CommandManager.INSTANCE.snapshot());
        MecDriveSubsystem.INSTANCE.sendTelemetry(UniConstants.loggingState.ENABLED);

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





}
