package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpModes.NextFTCTeleop;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class TurretSubsystem implements Subsystem {
    // put hardware, commands, etc here
    JoinedTelemetry telemetry;
    UniConstants.teamColor color = NextFTCTeleop.color;

    MotorEx launcher = new MotorEx(UniConstants.LAUNCHER_STRING).floatMode();

    int targetVelocity = 0;
    public static ControlSystem launcherControl;
    public static double p = .00022, i = 0, d = 0;

    //MotorEx turret = new MotorEx(UniConstants.TURRET_STRING).brakeMode().zeroed();
    public static double turretTargetAngle = 0;
    public static ControlSystem turretControl;


    public TurretSubsystem(){}



    @Override
    public void initialize() {
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());


        // initialization logic (runs on init)




        //TODO: how the freak does this work
//        turretControl = ControlSystem.builder()
//                .angular(AngleType.DEGREES,
//                        feedback -> feedback.posPid(.005, 0, 0)
//                )
//                .build();

    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        launcherControl = ControlSystem.builder()
                .velPid(p, i, d)
                .build();
        launcherControl.setLastMeasurement(new KineticState(0, launcher.getVelocity() * 2.1));
        launcher.setPower(launcherControl.calculate(new KineticState(0, targetVelocity)));
        //turret.setPower(turretControl.calculate(new KineticState(turretTargetAngle)));

    }

    public Command commandTargetAngleDegrees(double degrees){
        return new RunToPosition(turretControl, degrees, new KineticState(1.5));
    }

    public Command commandRunToVelocity(double velocity){
        return new RunToPosition(launcherControl, velocity);
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

    public void setTargetVelocity(int velo){
        targetVelocity = velo;
    }

    public void setTurretTargetAngleDegrees(double degrees){
        turretTargetAngle = degrees;
    }

    public void setTurretTargetAngleRadians(double radians){
        turretTargetAngle = Math.toDegrees(radians);
    }

    public double getCurrentVelocity(){
        return Math.abs(launcher.getVelocity() * 2.1);
    }

    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF OUTTAKE LOG");
                telemetry.addData("Turret Target Angle ", turretTargetAngle);
                telemetry.addData("Target Velocity ", targetVelocity);
                telemetry.addData("Current Velocity ", getCurrentVelocity());
                telemetry.addLine("END OF OUTTAKE LOG");
            case EXTREME:

                telemetry.addLine("START OF OUTTAKE LOG");

                telemetry.addLine("END OF OUTTAKE LOG");
        }
    }


}
