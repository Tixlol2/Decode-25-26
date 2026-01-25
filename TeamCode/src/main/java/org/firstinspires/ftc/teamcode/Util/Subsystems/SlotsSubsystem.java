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


    public SlotsSubsystem() {
        super(
                LeftSlot.INSTANCE,
                RightSlot.INSTANCE,
                BackSlot.INSTANCE
        );
    }

    private static final Timer shotTimer = new Timer();

    @Override
    public void initialize() {
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



}
