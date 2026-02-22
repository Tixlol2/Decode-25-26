package org.firstinspires.ftc.teamcode.OpModes;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
@Configurable
public class TestingOp extends NextFTCOpMode {


    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(RobotSubsystem.INSTANCE),
                BindingsComponent.INSTANCE

        );
    }

    public static double hoodTarget = 0;
    public static double flywheelTarget = 0;


    @Override
    public void onStartButtonPressed(){
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(RobotSubsystem.INSTANCE.Shoot());

        follower().setStartingPose(new Pose(32.5, 135.5, Math.toRadians(90)));
        follower().startTeleOpDrive();
        follower().update();
    }

    @Override
    public void onUpdate(){

        if(gamepad1.a){
            OuttakeSubsystem.maxRPM = flywheelTarget;
            OuttakeSubsystem.INSTANCE.setHood(hoodTarget);
        }
        if(gamepad1.b){
            OuttakeSubsystem.maxRPM = 0;
            OuttakeSubsystem.INSTANCE.setHood(.5);
        }

        //Spin active forward
        if (gamepad1.right_trigger > 0) {
            IntakeSubsystem.INSTANCE.setActiveState(IntakeSubsystem.IntakeState.OUT);
        } //Reverse Active and Spin
        else if (gamepad1.left_trigger > 0) {
            IntakeSubsystem.INSTANCE.setActiveState(IntakeSubsystem.IntakeState.IN);
        } else {
            IntakeSubsystem.INSTANCE.setActiveState(IntakeSubsystem.IntakeState.OFF);
        }

        OuttakeSubsystem.INSTANCE.sendTelemetry();
        telemetry.addData("Pose X: ", PedroComponent.follower().getPose().getX());
        telemetry.addData("Pose Y: ", PedroComponent.follower().getPose().getY());
        telemetry.addData("Distance to Goal: ", RobotSubsystem.INSTANCE.getDistanceToGoalInches());


    }
}
