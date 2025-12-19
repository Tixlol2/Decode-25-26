package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class TurretSubsystem implements Subsystem {
    // put hardware, commands, etc here
    JoinedTelemetry telemetry;
    UniConstants.teamColor color;

    private MotorEx launcher = new MotorEx(UniConstants.LAUNCHER_STRING).floatMode();
    private ControlSystem launcherControl;

    MotorEx turret = new MotorEx(UniConstants.TURRET_STRING).brakeMode().zeroed();
    public static double turretTargetAngle = 0;
    private ControlSystem turretControl;


    @Override
    public void initialize() {
        // initialization logic (runs on init)
        launcherControl = ControlSystem.builder()
                .velPid(.0001, 0, 0)
                .build();
        //TODO: how the fuck does this work
        turretControl = ControlSystem.builder()
                .angular(AngleType.DEGREES,
                        feedback -> feedback.posPid(.005, 0, 0)
                )
                .build();

    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)

        launcher.setPower(launcherControl.calculate(new KineticState(0, launcher.getVelocity())));
        turret.setPower(turretControl.calculate(new KineticState(turretTargetAngle)));

    }

    public Command commandTargetAngleDegrees(double degrees){
        return new RunToPosition(turretControl, degrees, new KineticState(1.5));
    }


    public  double getTargetVelocity(double distanceToGoalInMeters) {
        //https://www.desmos.com/calculator/yw7iis7m3w
        //https://medium.com/@vikramaditya.nishant/programming-a-decode-shooter-4ab114dac01f
        return Math.sqrt(
                ((9.81) * (Math.pow(distanceToGoalInMeters, 2)))
                        /
                        (Math.pow(2 * (Math.cos(Math.toRadians(UniConstants.ANGLE_OF_LAUNCHER_IN_DEGREES))), 2) * ((distanceToGoalInMeters * Math.tan(Math.toRadians(UniConstants.ANGLE_OF_LAUNCHER_IN_DEGREES))) - UniConstants.HEIGHT_TO_GOAL_WITH_CLEARANCE_METERS))
        );
    }

    public void setTargetVelocity(double velo){
        new RunToVelocity(launcherControl, velo);
    }

    public void setTurretTargetAngleDegrees(double degrees){
        turretTargetAngle = degrees;
    }

    public void setTurretTargetAngleRadians(double radians){
        turretTargetAngle = Math.toDegrees(radians);
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
