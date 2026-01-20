package org.firstinspires.ftc.teamcode.Util.Subsystems;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Util.Subsystems.Slots.BackSlot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Slots.LeftSlot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Slots.RightSlot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Slots.Slot;
import org.firstinspires.ftc.teamcode.Util.Timer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class SlotsSubsystem extends SubsystemGroup {

    public static final SlotsSubsystem INSTANCE = new SlotsSubsystem();
    public static Supplier<ArrayList<Slot>> result;
    private static final Timer shotTimer = new Timer();
    public ArrayList<Slot> slots = new ArrayList<>(Arrays.asList(LeftSlot.INSTANCE, RightSlot.INSTANCE, BackSlot.INSTANCE));

    public SlotsSubsystem() {
        super(
                LeftSlot.INSTANCE,
                RightSlot.INSTANCE,
                BackSlot.INSTANCE
        );
    }

    @Override
    public void initialize() {
        result = () -> determineOrder(Robot.patternSupplier.get());
    }

    @Override
    public void periodic() {


    }

    public Command SetAllSlotState(Slot.ServoState state) {
        return new ParallelGroup(
                LeftSlot.INSTANCE.setServoState(state),
                RightSlot.INSTANCE.setServoState(state),
                BackSlot.INSTANCE.setServoState(state)
        );
    }

    public ArrayList<Slot> determineOrder(@NonNull ArrayList<Slot.SlotState> pattern) {

        ArrayList<Slot> result1 = new ArrayList<>();
        Set<Slot> used = new HashSet<>();
        for (Slot.SlotState wanted : pattern) {
            for (Slot slot : slots) {
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
        for (Slot slot : slots) {
            if (slot.getColorState() != Slot.SlotState.EMPTY) {
                orderedResult.add(slot);
            }
        }

        // Then add empty slots
        for (Slot slot : slots) {
            if (slot.getColorState() == Slot.SlotState.EMPTY) {
                orderedResult.add(slot);
            }
        }

        if (orderedResult.size() < 3) {
            orderedResult = slots;
        }

        return orderedResult;
    }

    public Command Shoot() {

        Supplier<Slot> first = () -> result.get().get(0);
        Supplier<Slot> second = () -> result.get().get(1);
        Supplier<Slot> third = () -> result.get().get(2);


        return new SequentialGroup(

                new SequentialGroup(
                        new LambdaCommand()
                                .setStart(() -> first.get().basicShootDown().named("Result 1 " + first.get().getKickerServoName()).run())
                                .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting")),
                        new LambdaCommand()
                                .setStart(() -> second.get().basicShootDown().named("Result 2 " + second.get().getKickerServoName()).run())
                                .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting")),

                        new LambdaCommand()
                                .setStart(() -> third.get().basicShoot().named("Result 3 " + third.get().getKickerServoName()).run())
                                .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting"))

                ),
                new InstantCommand(() -> shotTimer.reset())
        ).setInterruptible(false).addRequirements(this);
    }

}
