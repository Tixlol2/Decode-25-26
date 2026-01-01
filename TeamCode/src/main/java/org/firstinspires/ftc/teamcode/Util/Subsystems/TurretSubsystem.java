package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class TurretSubsystem implements Subsystem {
    // put hardware, commands, etc here
    JoinedTelemetry telemetry;

    MotorEx launcher = new MotorEx(UniConstants.LAUNCHER_STRING).floatMode().reversed();

    public static int targetVelocity = 0;
    public static ControlSystem launcherControl;
    public static double pLaunch = .0035, iLaunch = 0, dLaunch = 0, fLaunch = 0, lLaunch = 0.13;
    private PDFLController launcherController = new PDFLController(pLaunch, dLaunch, fLaunch, lLaunch);
    private double launcherCurrentVelo = 0;



    MotorEx turret = new MotorEx(UniConstants.TURRET_STRING).floatMode().zeroed().brakeMode();
    public static double turretTargetAngle = 0;
    private double heading = 0;
    private double turretCurrentPos = 0;
    private final double pTurret = 0.003, dTurret = 0, lTurret = 0.125, fTurret = 0;
    private final PDFLController turretControl = new PDFLController(pTurret, dTurret, fTurret, lTurret);


    public TurretSubsystem(){}



    @Override
    public void initialize() {
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        init();

    }

    @Override
    public void periodic() {
        if(!ActiveOpMode.opModeInInit()) {
            // periodic logic (runs every loop)

            //NextFTC Control System PID
//        launcherControl = ControlSystem.builder()
//                .velPid(p)
//                .build();
//        launcherControl.setLastMeasurement(new KineticState(0, launcher.getVelocity() * 2.1));
//        launcher.setPower(launcherControl.calculate(new KineticState(0, targetVelocity)));

            launcherController.setPDFL(pLaunch, dLaunch, fLaunch, lLaunch);
            launcherCurrentVelo = getCurrentVelocity();
            launcherController.setTarget(targetVelocity);
            launcherController.update(launcherCurrentVelo);
            launcher.setPower(Math.max(0.0, Math.min(1.0, launcherController.runPDFL(50))));


            //turretControl.setPDFL(pTurret, dTurret, fTurret, lTurret);
            turretCurrentPos = turret.getCurrentPosition();
            turretTargetAngle = Math.max(-65.0, Math.min(65, turretTargetAngle));
            //turretControl.setTarget(Math.max(angleToTicks(-65.0), Math.min(angleToTicks(65), angleToTicks(turretTargetAngle) + angleToTicks(heading))));
            turretControl.update(turretCurrentPos);
            turret.setPower(turretControl.runPDFL(angleToTicks(.5)));
        }
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

    public void setTargetAngle(double angleDeg){
        turretTargetAngle = angleDeg;
    }

    //Uses degrees
    public double angleToTicks(double angle){
        return angle * UniConstants.TURRET_TICKS_PER_DEGREE;
    }

    //Uses degrees
    public double ticksToAngle(double ticks){
        return (ticks / UniConstants.TURRET_TICKS_PER_DEGREE) % 360;
    }

    public double getCurrentVelocity(){
        return Math.abs(launcher.getVelocity() * 2.1);
    }

    public void init(){
        turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretTargetAngle = 0;
        targetVelocity = 0;
    }


    public double getTurretTargetAngle(){
        return turretTargetAngle;
    }

    public void setHeading(double heading){
        this.heading = heading;
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
                telemetry.addLine();
                telemetry.addData("Turret Position Deg ", ticksToAngle(turretCurrentPos));
                telemetry.addData("Turret Position Ticks ", (turretCurrentPos));
                telemetry.addData("Turret Target Deg ", turretTargetAngle);



                telemetry.addLine("END OF OUTTAKE LOG");
            case EXTREME:

                telemetry.addLine("START OF OUTTAKE LOG");

                telemetry.addLine("END OF OUTTAKE LOG");
        }
    }


}
