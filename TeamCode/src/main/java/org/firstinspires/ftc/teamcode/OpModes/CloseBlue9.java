package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.AutonUtil.CloseBlue9Paths;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "9 Ball Blue Close", group = "Main")
public class CloseBlue9 extends NextFTCOpMode {
    {
        addComponents(
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    CloseBlue9Paths paths;

    JoinedTelemetry joinedTelemetry;

    int pathState = 0;
    Timer pathTimer = new Timer();


    public static double creepDistance = 2.5;



    @Override
    public void onInit(){
        joinedTelemetry = Robot.INSTANCE.getJoinedTelemetry();
        paths = new CloseBlue9Paths(MecDriveSubsystem.INSTANCE.getFollower());
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

    @Override
    public void onStartButtonPressed(){

        new SequentialGroup(
                TurretSubsystem.INSTANCE.SetMotorPower(.65),
                Robot.INSTANCE.TurretObelisk(),
                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path1, true),
                Robot.INSTANCE.FaceGoal(),
                Robot.INSTANCE.ShootCreep(creepDistance, .5),

                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path2, false),

                Robot.INSTANCE.FollowPathActive(paths.Path3, false, .85),

                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path4, true),
                Robot.INSTANCE.ShootCreep(creepDistance, .5),

                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path5, false),

                Robot.INSTANCE.FollowPathActive(paths.Path6, false, .85),

                MecDriveSubsystem.INSTANCE.FollowPath(paths.Path7, true),
                Robot.INSTANCE.ShootCreep(creepDistance, .5),

                new ParallelGroup(
                    MecDriveSubsystem.INSTANCE.FollowPath(paths.Path8, true),
                        Robot.INSTANCE.StopSubsystems()
                )
        ).schedule();


    }

    @Override
    public void onUpdate(){
        joinedTelemetry.addData("Path State: ", pathState);
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
