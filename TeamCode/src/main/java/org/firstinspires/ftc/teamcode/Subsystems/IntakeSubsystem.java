package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class IntakeSubsystem implements Subsystem {

    public static final IntakeSubsystem INSTANCE = new IntakeSubsystem();
    public final MotorEx active = new MotorEx(UniConstants.ACTIVE_INTAKE_STRING).floatMode();
    private IntakeState activeState = IntakeState.OFF;

    @Override
    public void periodic() {
        if (ActiveOpMode.isStarted()) {
            active.setPower(activeState != IntakeState.OFF ? (activeState == IntakeState.IN ? 1 : -1) : 0);
        }
    }

    public void setActiveState(IntakeState state) {
        activeState = state;
    }

    public Command setActiveStateCommand(IntakeState state) {
        return new InstantCommand(() -> setActiveState(state));
    }

    public Command runActive(){
        return new LambdaCommand()
                .setStart(() -> setActiveState(IntakeState.IN))
                .setStop((interrupted) -> setActiveState(IntakeState.OFF))
                .setIsDone(() -> false);
    }



    public enum IntakeState {
        IN,
        OUT,
        OFF
    }


}
