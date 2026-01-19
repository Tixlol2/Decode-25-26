package org.firstinspires.ftc.teamcode.OpModes;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public class Auto extends NextFTCOpMode {


    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Robot.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        ); //Subsystems
    }

    JoinedTelemetry joinedTelemetry;
    private int pathState = 0;

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

        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("Circle for Blue, X for Red ");
        joinedTelemetry.addData("Current Team Color ", Robot.color);
    }

    @Override
    public void onStartButtonPressed() {
        follower().setStartingPose(Robot.color == Robot.teamColor.BLUE ? Poses.blueGoalTopStartFacing : Poses.redGoalTopStartFacing);
        Robot.INSTANCE.setGlobalColor();
        setPathState(0);
    }

    @Override
    public void onUpdate() {
        Robot.previousPose = follower().getPose();
    }




    public void setPathState(int state) {
        pathState = state;
    }

    public Command SetPathState(int state) {
        return new InstantCommand(() -> setPathState(state));
    }

    public void pathUpdate() {

        switch (pathState) {
            case 0:
                break;
        }


    }

}
