package org.firstinspires.ftc.teamcode.OpModes;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Slots.BackSlot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Slots.LeftSlot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Slots.RightSlot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Slots.Slot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.SlotsSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
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

    public static Supplier<ArrayList<Slot>> result;


    @Override
    public void onInit() {
        joinedTelemetry = Robot.INSTANCE.getJoinedTelemetry();
        result = () -> determineOrder(Robot.patternSupplier.get());

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
        follower().setStartingPose(Robot.previousPose);
        follower().startTeleopDrive();
        Robot.INSTANCE.setGlobalColor();
        createBindings();

    }

    @Override
    public void onUpdate() {
        Robot.previousPose = follower().getPose();

        isSlowed = gamepad1.left_bumper;

        //Spin active forward
        if (gamepad1.right_trigger > 0) {
            IntakeSubsystem.INSTANCE.forwardIntake();
        } //Reverse Active and Spin
        else if (gamepad1.left_trigger > 0) {
            IntakeSubsystem.INSTANCE.reverseIntake();
        } else {
            IntakeSubsystem.INSTANCE.disableActive();
        }

        //Kill button
        if ((gamepad1.yWasPressed())) {
            CommandManager.INSTANCE.cancelAll();
            SlotsSubsystem.INSTANCE.SetAllSlotState(Slot.ServoState.DOWN).schedule();
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
        //IntakeSortingSubsystem.INSTANCE.sendTelemetry(Robot.loggingState.ENABLED);
        joinedTelemetry.addData("Commands: ", CommandManager.INSTANCE.snapshot());


    }

    void createBindings() {


        //Toggle things based on dpad
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(TurretSubsystem.INSTANCE.TurretForward());
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(TurretSubsystem.INSTANCE.TurretGoal());
        //Gamepads.gamepad1().dpadRight().whenBecomesTrue(TurretSubsystem.INSTANCE::init);
        //Gamepads.gamepad1().dpadDown().whenBecomesTrue(SlotsSubsystem.INSTANCE.SetAllSlotState(Slot.ServoState.DOWN));


        //Face buttons
        Gamepads.gamepad1().a().whenBecomesTrue(TurretSubsystem.INSTANCE.SetFlywheelState(TurretSubsystem.FlywheelState.SHORT));
        Gamepads.gamepad1().b().whenBecomesTrue(TurretSubsystem.INSTANCE.SetFlywheelState(TurretSubsystem.FlywheelState.OFF));
        Gamepads.gamepad1().x().whenBecomesTrue(TurretSubsystem.INSTANCE.SetFlywheelState(TurretSubsystem.FlywheelState.FAR));

        //Shooting command
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(Shoot());


    }

    public ArrayList<Slot> determineOrder(@NonNull ArrayList<Slot.SlotState> pattern) {

        ArrayList<Slot> result1 = new ArrayList<>();
        Set<Slot> used = new HashSet<>();
        for (Slot.SlotState wanted : pattern) {
            for (Slot slot : Robot.INSTANCE.slots) {
                if (slot.getColorState() == wanted && used.add(slot)) {
                    result1.add(slot);
                    break;
                }
            }
        }

        if (result1.size() >= 3) {
            return result1;
        }

        // Not full - add slots that are full first, then empty ones
        ArrayList<Slot> orderedResult = new ArrayList<>();

        // Add full slots first
        for (Slot slot : Robot.INSTANCE.slots) {
            if (slot.getColorState() != Slot.SlotState.EMPTY) {
                orderedResult.add(slot);
            }
        }

        // Then add empty slots
        for (Slot slot : Robot.INSTANCE.slots) {
            if (slot.getColorState() == Slot.SlotState.EMPTY) {
                orderedResult.add(slot);
            }
        }

        return orderedResult;
    }

    public Command Shoot() {

        Supplier<Slot> first = () -> result.get().get(0);
        Supplier<Slot> second = () -> result.get().get(1);
        Supplier<Slot> third = () -> result.get().get(2);


        return new SequentialGroup(
                new ParallelGroup(
                        new LambdaCommand()
                                .setStart(() -> first.get().basicShootDown().named("Result 1 " + first.get().getKickerServoName()).run())
                                .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting")),
                        new Delay(1)),
                        new LambdaCommand()
                                .setStart(() -> second.get().basicShootDown().named("Result 2 " + second.get().getKickerServoName()).run())
                                .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting")),

                        new LambdaCommand()
                                .setStart(() -> third.get().basicShoot().named("Result 3 " + third.get().getKickerServoName()).run())
                                .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting"))


                //new InstantCommand(shotTimer::reset)
        ).setInterruptible(false).addRequirements(this);
    }


}
