package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class TurretSubsystem implements Subsystem {
    // put hardware, commands, etc here
    JoinedTelemetry telemetry;
    UniConstants.teamColor color;

    private MotorEx launcher = new MotorEx(UniConstants.LAUNCHER_STRING)
            .floatMode();

    private ControlSystem launcherControl;



    @Override
    public void initialize() {
        // initialization logic (runs on init)
        launcherControl = ControlSystem.builder()
                .velPid(.0001, 0, 0)
                .build();
        launcherControl.setGoal(new KineticState(0, 1000));

    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)

        launcher.setPower(
                launcherControl.calculate(
                        new KineticState(0, launcher.getVelocity())
                ));
        new RunToVelocity(launcherControl, 1000);

    }


    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF OUTTAKE LOG");

                telemetry.addLine("END OF OUTTAKE LOG");
            case EXTREME:

                telemetry.addLine("START OF OUTTAKE LOG");

                telemetry.addLine("END OF OUTTAKE LOG");
        }
    }


}
