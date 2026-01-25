package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Subsystems.Slots.BackSlot;
import org.firstinspires.ftc.teamcode.Subsystems.Slots.LeftSlot;
import org.firstinspires.ftc.teamcode.Subsystems.Slots.MainSlot;
import org.firstinspires.ftc.teamcode.Subsystems.Slots.RightSlot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class SortingSubsystem extends SubsystemGroup {
    
    public static final SortingSubsystem INSTANCE = new SortingSubsystem();
    
    public SortingSubsystem(){
        super(
                BackSlot.INSTANCE,
                LeftSlot.INSTANCE,
                RightSlot.INSTANCE
        );
    }
    
    private static Supplier<ArrayList<MainSlot>> result;
    private static final ArrayList<MainSlot> slots = new ArrayList<>(Arrays.asList(BackSlot.INSTANCE, LeftSlot.INSTANCE, RightSlot.INSTANCE));

    @Override
    public void initialize() {
        result = () -> determineOrder(RobotSubsystem.patternSupplier.get());
    }

    @Override
    public void periodic() {
        
    }

    public ArrayList<MainSlot> determineOrder(@NonNull ArrayList<MainSlot.SlotState> pattern) {

        ArrayList<MainSlot> result1 = new ArrayList<>();
        Set<MainSlot> used = new HashSet<>();
        for (MainSlot.SlotState wanted : pattern) {
            for (MainSlot slot : slots) {
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
        ArrayList<MainSlot> orderedResult = new ArrayList<>();

        // Add full slots first
        for (MainSlot slot : slots) {
            if (slot.getColorState() != MainSlot.SlotState.EMPTY) {
                orderedResult.add(slot);
            }
        }

        // Then add empty slots
        for (MainSlot slot : slots) {
            if (slot.getColorState() == MainSlot.SlotState.EMPTY) {
                orderedResult.add(slot);
            }
        }

        return orderedResult;
    }

    public Command Shoot() {

        Supplier<MainSlot> first = () -> result.get().get(0);
        Supplier<MainSlot> second = () -> result.get().get(1);
        Supplier<MainSlot> third = () -> result.get().get(2);


        return new SequentialGroup(
                new ParallelGroup(
                        new LambdaCommand()
                                .setStart(() -> first.get().basicShootDown().named("Result 1 " + first.get().getKickerServoName()).run())
                                .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting")),
                new LambdaCommand()
                        .setStart(() -> second.get().basicShootDown().named("Result 2 " + second.get().getKickerServoName()).run())
                        .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting")),

                new LambdaCommand()
                        .setStart(() -> third.get().basicShoot().named("Result 3 " + third.get().getKickerServoName()).run())
                        .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting"))


                //new InstantCommand(shotTimer::reset)
        ).setInterruptible(false).addRequirements(this));
    }
    
    
    public Command SetAllSlotState(MainSlot.ServoState state) {
        return new ParallelGroup(
                LeftSlot.INSTANCE.setServoState(state),
                RightSlot.INSTANCE.setServoState(state),
                BackSlot.INSTANCE.setServoState(state)
        );
    }
    
}
