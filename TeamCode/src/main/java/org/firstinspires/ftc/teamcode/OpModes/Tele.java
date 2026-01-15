package org.firstinspires.ftc.teamcode.OpModes;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSortingSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

//Written by Noah Nottingham - 6566 Circuit Breakers

@TeleOp(name = "Tele", group = "Main") //The name and group
@Configurable
public class Tele extends NextFTCOpMode {
    public static double debugPower = .4;
    private final boolean botCentric = true;
    private final boolean enableRumble = false;
    JoinedTelemetry joinedTelemetry;
    Timer rumblingTimer = new Timer();
    private boolean isSlowed = false;

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
        follower().startTeleopDrive();
        Robot.INSTANCE.setGlobalColor();
        createBindings();
        follower().setStartingPose(Auto.endPose);


    }

    @Override
    public void onUpdate() {

        isSlowed = gamepad1.left_bumper;

        //Spin active forward
        if (gamepad1.right_trigger > 0) {
            IntakeSortingSubsystem.INSTANCE.forwardIntake();
        } //Reverse Active and Spin
        else if (gamepad1.left_trigger > 0) {
            IntakeSortingSubsystem.INSTANCE.reverseIntake();
        } else {
            IntakeSortingSubsystem.INSTANCE.disableActive();
        }

        //Rumble control
        if (IntakeSortingSubsystem.INSTANCE.shouldRumble() && rumblingTimer.getTimeSeconds() > 3 && enableRumble) {
            gamepad1.rumble(1000);
            rumblingTimer.reset();
        }

        //Kill button
        if ((gamepad1.yWasPressed())) {
            CommandManager.INSTANCE.cancelAll();
            follower().startTeleopDrive();
            Robot.automatedDrive = false;
        }

        //Driver controlled
        if (!Robot.automatedDrive) {
            follower().setTeleOpDrive(
                    -gamepad1.left_stick_y * (isSlowed ? .25 : 1),
                    -gamepad1.left_stick_x * (isSlowed ? .25 : 1),
                    -gamepad1.right_stick_x * (isSlowed ? .25 : 1),
                    botCentric
            );
        }


        TurretSubsystem.INSTANCE.sendTelemetry(Robot.loggingState.ENABLED);
        MecDriveSubsystem.INSTANCE.sendTelemetry(Robot.loggingState.ENABLED);
        IntakeSortingSubsystem.INSTANCE.sendTelemetry(Robot.loggingState.ENABLED);
        joinedTelemetry.addData("Commands: ", CommandManager.INSTANCE.snapshot());


    }

    void createBindings() {

        //Disable active when triggers not held down

        //Toggle things based on dpad
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> TurretSubsystem.INSTANCE.TurretForward().schedule());
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> TurretSubsystem.INSTANCE.TurretGoal().schedule());
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(Robot.INSTANCE.Park());
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> {
            follower().setStartingPose(Poses.blueGoalTopStartFacing);
        });


        //Face buttons
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            TurretSubsystem.INSTANCE.setMotorPower(.65);
        });
        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            TurretSubsystem.INSTANCE.setMotorPower(0);
        });
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            TurretSubsystem.INSTANCE.setMotorPower(debugPower);
        });

        //Shooting command
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(IntakeSortingSubsystem.INSTANCE.Shoot());


    }


}
