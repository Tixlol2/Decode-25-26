package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class IntakeSubsystem implements Subsystem {

    public static IntakeSubsystem INSTANCE = new IntakeSubsystem();
    public static boolean isReversed = false;
    public boolean isEnabled = false;
    //Class variables
    JoinedTelemetry telemetry;
    //Active motor
    MotorEx active = new MotorEx(UniConstants.ACTIVE_INTAKE_STRING).floatMode().reversed();

    public IntakeSubsystem() {
    }

    @Override
    public void initialize() {


    }

    @Override
    public void periodic() {

        if (ActiveOpMode.isStarted()) {
            new SetPower(active, isEnabled ? (isReversed ? -1 : 1) : 0).run();
        }

    }

    //Active Methods
    public void disableActive() {
        isEnabled = false;
    }

    public void enableActive() {
        isEnabled = true;
    }

    public void reverseIntake() {
        isReversed = true;
        enableActive();
    }

    public void forwardIntake() {
        isReversed = false;
        enableActive();
    }


    public Command runActive() {
        return new InstantCommand(() -> {
            forwardIntake();
            enableActive();
        });
    }

    public Command reverseActive() {
        return new InstantCommand(() -> {
            reverseIntake();
            enableActive();
        });
    }

    public Command stopActive() {
        return new InstantCommand(this::disableActive);
    }

    public void sendTelemetry(Robot.loggingState state) {
        switch (state) {
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF SORTING LOG");
                telemetry.addData("Intake Enabled ", isEnabled);
                telemetry.addData("Intake Reversed ", isReversed);


                telemetry.addLine("END OF SORTING LOG");
                telemetry.addLine();
                break;
            case EXTREME:
                telemetry.addLine("START OF ROTARY LOG");
                telemetry.addLine("END OF ROTARY LOG");


        }
    }

    public void setTelemetry(JoinedTelemetry telemetry) {
        this.telemetry = telemetry;
    }


}