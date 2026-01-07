package org.firstinspires.ftc.teamcode.Util;

import java.util.function.BooleanSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;

//Written by Ernest | 17089 on NextFTC Discord, cheezburgir
public class IfElseCommand extends Command {
    private final BooleanSupplier condition;
    private final Command trueCommand;
    private final Command falseCommand;
    private Command selectedCommand;

    public IfElseCommand(BooleanSupplier condition, Command trueCommand) {
        this(condition, trueCommand, new NullCommand());
    }

    public IfElseCommand(BooleanSupplier condition, Command trueCommand, Command falseCommand) {
        this.condition = condition;
        this.trueCommand = trueCommand;
        this.falseCommand = falseCommand;
    }

    @Override
    public boolean isDone() {
        return selectedCommand.isDone();
    }

    @Override
    public void start() {
        selectedCommand = condition.getAsBoolean() ? trueCommand : falseCommand;
        selectedCommand.start();
    }

    @Override
    public void update() {
        selectedCommand.update();
    }

    @Override
    public void stop(boolean interrupted) {
        selectedCommand.stop(interrupted);
    }
}