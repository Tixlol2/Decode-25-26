package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
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
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "9 Ball Blue Close Wait", group = "Main")
public class CloseBlue9Wait extends NextFTCOpMode {
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
        //joinedTelemetry = Robot.INSTANCE.getJoinedTelemetry();
        joinedTelemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
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
                new ParallelGroup(
                    TurretSubsystem.INSTANCE.SetMotorPower(.65),
                    Robot.INSTANCE.TurretObelisk(),
                    MecDriveSubsystem.INSTANCE.FollowPath(paths.ScanAndShoot, true)
                ),
                Robot.INSTANCE.FaceGoal(),
                Robot.INSTANCE.ShootWait(.5),

                ChangePath(),

                MecDriveSubsystem.INSTANCE.FollowPath(paths.MoveToActive1, false),

                ChangePath(),

                Robot.INSTANCE.FollowPathActive(paths.Active1, false, .85),

                ChangePath(),

                MecDriveSubsystem.INSTANCE.FollowPath(paths.Shoot2, true),
                Robot.INSTANCE.ShootWait(.5),

                ChangePath(),

                MecDriveSubsystem.INSTANCE.FollowPath(paths.MovetoActive2, false),

                ChangePath(),

                Robot.INSTANCE.FollowPathActive(paths.Active2, false, .85),

                ChangePath(),

                MecDriveSubsystem.INSTANCE.FollowPath(paths.Shoot3, true),
                Robot.INSTANCE.ShootWait(.5),

                ChangePath(),

                new ParallelGroup(
                    MecDriveSubsystem.INSTANCE.FollowPath(paths.Park, true),
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
