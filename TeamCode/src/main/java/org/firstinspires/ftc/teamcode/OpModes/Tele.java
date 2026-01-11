package org.firstinspires.ftc.teamcode.OpModes;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSortingSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
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

    public static double debugPower = .9;
    JoinedTelemetry joinedTelemetry;
    Timer rumblingTimer = new Timer();
    private boolean isSlowed = false;
    private boolean autoShoot = true;
    private final boolean botCentric = true;
    private final boolean enableRumble = false;

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
            Robot.color = UniConstants.teamColor.RED;
        } else if (gamepad1.b) {
            Robot.color = UniConstants.teamColor.BLUE;
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
        autoShoot = true;


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

        if (IntakeSortingSubsystem.INSTANCE.shouldRumble() && rumblingTimer.getTimeSeconds() > 3 && enableRumble) {
            gamepad1.rumble(1000);
            rumblingTimer.reset();
        }

        if ((gamepad1.yWasPressed())) {
            CommandManager.INSTANCE.cancelAll();
            follower().startTeleopDrive();
            Robot.automatedDrive = false;
        }

        if (!Robot.automatedDrive) {
            follower().setTeleOpDrive(
                    -gamepad1.left_stick_y * (isSlowed ? .25 : 1),
                    -gamepad1.left_stick_x * (isSlowed ? .25 : 1),
                    -gamepad1.right_stick_x * (isSlowed ? .25 : 1),
                    botCentric
            );
        }

//        if(gamepad1.right_bumper){
//            new SequentialGroup(
//                    Robot.INSTANCE.ShootWait(),
//                    new InstantCommand(() -> {follower().breakFollowing(); follower().startTeleopDrive();})
//            ).schedule();
//
//        }

        Robot.order = IntakeSortingSubsystem.INSTANCE.determineOrder(Robot.pattern);


        //TODO: Still have to integrate look up table or linreg for power as a function of distance

        joinedTelemetry.addData("Bot Centric: ", botCentric);
        joinedTelemetry.addData("Automated Shooting: ", autoShoot);
        //joinedTelemetry.addData("Current Commands: ", CommandManager.INSTANCE.snapshot());
        TurretSubsystem.INSTANCE.sendTelemetry(UniConstants.loggingState.ENABLED);
        MecDriveSubsystem.INSTANCE.sendTelemetry(UniConstants.loggingState.ENABLED);
        IntakeSortingSubsystem.INSTANCE.sendTelemetry(UniConstants.loggingState.ENABLED);


    }

    void createBindings() {

        //Disable active when triggers not held down
        //Gamepads.gamepad1().rightTrigger().atMost(.1).and(Gamepads.gamepad1().leftTrigger().atMost(.1)).whenFalse(IntakeSortingSubsystem.INSTANCE::disableActive);

        //Run active in direction based on trigger
//        Gamepads.gamepad1().rightTrigger().greaterThan(0).whenTrue(
//                () -> {
//                    IntakeSortingSubsystem.INSTANCE.enableActive();
//                    IntakeSortingSubsystem.INSTANCE.forwardIntake();
//                });
//
//        Gamepads.gamepad1().leftTrigger().greaterThan(0).whenTrue(
//                () -> {
//                    IntakeSortingSubsystem.INSTANCE.enableActive();
//                    IntakeSortingSubsystem.INSTANCE.reverseIntake();
//                });

        //Toggle things based on dpad
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> Robot.INSTANCE.TurretForward().schedule());
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> Robot.INSTANCE.TurretGoal().schedule());
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(Robot.INSTANCE.Park());
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> {
            TurretSubsystem.INSTANCE.setMotorPower(-.7);
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
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(Robot::shootTest);


    }


}
