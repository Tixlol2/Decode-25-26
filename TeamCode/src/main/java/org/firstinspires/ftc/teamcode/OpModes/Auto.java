package org.firstinspires.ftc.teamcode.OpModes;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.telemetry.JoinedTelemetry;

import org.firstinspires.ftc.teamcode.OpModes.Paths.Close6;
import org.firstinspires.ftc.teamcode.OpModes.Paths.Close9;
import org.firstinspires.ftc.teamcode.OpModes.Paths.MainPaths;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public class Auto extends NextFTCOpMode {


    JoinedTelemetry joinedTelemetry;
    MainPaths paths = new Close6();
    private int pathState = 0;
    private int oldPathState = 0;

    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        ); //Subsystems
    }

    @Override
    public void onInit() {
        joinedTelemetry = Robot.INSTANCE.getJoinedTelemetry();
        Robot.inTeleop = false;
        TurretSubsystem.INSTANCE.init();
    }

    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            Robot.color = Robot.teamColor.RED;
        } else if (gamepad1.b) {
            Robot.color = Robot.teamColor.BLUE;
        }

        if (gamepad1.dpad_up) {
            paths = new Close6();
        } else if (gamepad1.dpad_down){
            paths = new Close9();
        }

        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("Circle for Blue, X for Red ");
        joinedTelemetry.addData("Current Team Color ", Robot.color);
        joinedTelemetry.addData("Current Path ", paths.name);
    }

    @Override
    public void onStartButtonPressed() {
        follower().setStartingPose(paths.startingPose);
        Robot.INSTANCE.setGlobalColor();
        setPathState(0);
    }

    @Override
    public void onUpdate() {
        Robot.previousPose = follower().getPose();
        if (pathState != oldPathState) {
            oldPathState = pathState;
            pathUpdate();
        }
    }


    public void setPathState(int state) {
        if (paths.numCommands >= state) {
            pathState = state;
        } else {
            pathState = -1;
        }

    }

    public Command SetPathState(int state) {
        return new InstantCommand(() -> setPathState(state));
    }

    public void pathUpdate() {

        switch (pathState) {
            case -1:
                break;
            case 0:
                paths.command1.schedule();
                if (paths.command1.isDone()) {
                    setPathState(1);
                }
                break;
            case 1:
                paths.command2.schedule();
                if (paths.command2.isDone()) {
                    setPathState(2);
                }
                break;
            case 2:
                paths.command3.schedule();
                if (paths.command3.isDone()) {
                    setPathState(3);
                }
                break;
            case 3:
                paths.command4.schedule();
                if (paths.command4.isDone()) {
                    setPathState(4);
                }
                break;
            case 4:
                paths.command5.schedule();
                if (paths.command5.isDone()) {
                    setPathState(5);
                }
                break;
            case 5:
                paths.command6.schedule();
                if (paths.command6.isDone()) {
                    setPathState(6);
                }
                break;
            case 6:
                paths.command7.schedule();
                if (paths.command7.isDone()) {
                    setPathState(7);
                }
                break;

        }


    }

}
