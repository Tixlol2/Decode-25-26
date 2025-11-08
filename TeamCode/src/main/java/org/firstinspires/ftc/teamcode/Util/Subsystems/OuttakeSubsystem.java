package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.subsystems.Subsystem;

@Configurable
public class OuttakeSubsystem implements Subsystem {
    //Necessary
    JoinedTelemetry telemetry;
    UniConstants.teamColor color;
    public static boolean debug = false;

    //Actual launcher things
    DcMotorEx launcher;
    public static double pL = 0, dL = 0, lL = 0, fL = 0;
    PDFLController launcherController = new PDFLController(pL, dL, fL, lL);
    private static double launcherTargetVelo = 0;
    private static double launcherCurrentVelo = 0;
    private static double launcherPower = 0;
    //Debug launcher things
    public static double launcherTargetVeloDebug = 0;
    public static double launcherPowerDebug = 0;

    //Actual turret things
    DcMotorEx turret;
    public static double pT = 0, dT = 0, lT = 0, fT = 0;
    PDFLController turretController = new PDFLController(pT, dT, fT, lT);
    private static double turretTargetPosition = 0;
    private static double turretCurrentPositon = 0;
    private static double turretTargetAngle = 0;
    private static double turretCurrentAngle = 0;
    //Debug turret things
    public static double turretPower = 0;
    private static double turretTargetAngleDebug = 0;





    public OuttakeSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry, UniConstants.teamColor color){
        this.telemetry = telemetry;
        this.color = color;

        launcher = hardwareMap.get(DcMotorEx.class, UniConstants.LAUNCHER_STRING);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret = hardwareMap.get(DcMotorEx.class, UniConstants.TURRET_ROTATION_STRING);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void periodic(){


        turretController.setPDFL(pT, dT, fT, lT);
        launcherController.setPDFL(pL, dL, fL, lL);
        if(debug) {
            setTurretTargetAngle(turretTargetAngleDebug);
        }
        launcherCurrentVelo = launcher.getVelocity();

        turretCurrentPositon = turret.getCurrentPosition();
        turretCurrentAngle = getTurretCurrentAngle(turretCurrentPositon);


        //TODO: Needs to be tuned for PDFL
        turretController.setTarget(turretTargetPosition);
        turretController.update(turretCurrentPositon);
        turret.setPower(debug ? turretPower : turretController.runPDFL(5));

        //TODO: Tune for PDFL
        launcherController.setTarget(debug ? launcherTargetVeloDebug : launcherTargetVelo);
        launcherController.update(launcherCurrentVelo);
        launcher.setPower(debug ? launcherPowerDebug : (launcherPower += launcherController.runPDFL(.05)));




        turretTargetPosition = getTurretTargetPosition(turretTargetAngle);


    }

    public  double getTurretTargetPosition(double turretTargetAngle){

        //Ratio given in terms of motor/turret

        return Math.max(Math.min(-400,turretTargetAngle * UniConstants.TURRET_TICKS_PER_DEGREE),400);

    }

    public  double getTurretCurrentAngle(double turretCurrentPositon){

        //Ratio given in terms of motor/turret

        return (turretCurrentPositon * UniConstants.MOTOR_TO_TURRET_RATIO) / UniConstants.TURRET_TICKS_PER_DEGREE;

    }

    public  void setTurretTargetAngle(double target){
        turretTargetAngle = Math.max(35,Math.min(debug ? turretTargetAngleDebug :target, 35));
    }

    public void resetMotors(){
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public  void setLauncherTargetVelo(double target){
        launcherTargetVelo = target;
    }

    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF OUTTAKE LOG");
                telemetry.addData("Launcher Target Velo ", launcherTargetVelo);
                telemetry.addData("Launcher Current Velo ", launcherCurrentVelo);
                telemetry.addLine();
                telemetry.addData("Turret Target Pos ", turretTargetPosition);
                telemetry.addData("Turret Current Pos ", turretCurrentPositon);
                telemetry.addData("Turret Target Angle ", turretTargetAngle);
                telemetry.addData("Turret Current Angle ", turretCurrentAngle);
                telemetry.addLine("END OF OUTTAKE LOG");
            case EXTREME:

                telemetry.addLine("START OF OUTTAKE LOG");
                telemetry.addData("Outtake Debug ", debug);
                telemetry.addData("Launcher Target Velo ", launcherTargetVelo);
                telemetry.addData("Launcher Current Velo ", launcherCurrentVelo);
                telemetry.addLine();
                telemetry.addData("Turret Target Pos ", turretTargetPosition);
                telemetry.addData("Turret Current Pos ", turretCurrentPositon);
                telemetry.addData("Turret Target Angle ", turretTargetAngle);
                telemetry.addData("Turret Current Angle ", turretCurrentAngle);
                telemetry.addLine("END OF OUTTAKE LOG");
        }
    }


}
