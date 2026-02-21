package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Configurable
public class OuttakeSubsystem implements Subsystem {

    public static final OuttakeSubsystem INSTANCE = new OuttakeSubsystem();

    //Misc. Stuffs
    public static boolean debug = false;
    public static double debugPower = 0;
    public static boolean turretEnabled = true;

    //Launcher Stuffs
    private static final MotorEx leftLaunchMotor = new MotorEx(UniConstants.LAUNCHER_LEFT_STRING).floatMode().reversed();
    private static final MotorEx rightLaunchMotor = new MotorEx(UniConstants.LAUNCHER_RIGHT_STRING).floatMode();
    private static final MotorGroup launcherGroup = new MotorGroup(rightLaunchMotor, leftLaunchMotor); //Right has encoder, put first
    private static FlywheelState launcherState = FlywheelState.OFF;
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(.017, 0, 0); //TODO: Tune
    private static ControlSystem launcherControl;
    public static double targetVeloRPM = 0;

    //Hood Stuffs
    ServoEx hood = new ServoEx("HOOD");
    private static double hoodTargetPosition = .5;
    public static double hoodLinreg = 0;
    public static double debugHoodTargetPosition = .75;

    //Turret Stuffs
    private static final MotorEx turret = new MotorEx(UniConstants.TURRET_STRING).zeroed().brakeMode();
    private static TurretState turretState = TurretState.FORWARD;
    public static double pTurret = .00145, dTurret = 0, lTurret = 0.15, fTurret = 0;
    private PDFLController turretControl = new PDFLController(pTurret, dTurret, fTurret, lTurret);
    private static double turretTargetAngle = 0;
    public static double turretAngleTolerance = .5;

    public static double debugTargetAngle = 0;

    public static double midRPM = 2750;
    public static double maxRPM = 3750;


    @Override
    public void initialize() {
        launcherControl = ControlSystem.builder()
                .velPid(launcherPIDCoefficients)
                .build();
        turretControl.setPDFL(pTurret, dTurret, fTurret, lTurret);
    }

    @Override
    public void periodic() {
        if (ActiveOpMode.isStarted()) {
            //Flywheel control
            if (!debug) {
                switch (launcherState) {
                    case OFF:
                        launcherControl.setGoal(new KineticState(0, toTicksPerSec(0)));
                        break;
                    case MEDIUM:
                        launcherControl.setGoal(new KineticState(0, toTicksPerSec(midRPM)));
                        break;
                    case FULL:
                        launcherControl.setGoal(new KineticState(0, toTicksPerSec(maxRPM)));
                        break;
                }
                launcherGroup.setPower(Math.max(0, Math.min(1, launcherControl.calculate(
                                new KineticState(launcherGroup.getCurrentPosition(), launcherGroup.getVelocity())
                        )))
                );
            } else {
                launcherGroup.setPower(debugPower);
            }

            //Turret control
            if (turretEnabled) {
                if (debug) {
                    turretControl.setPDFL(pTurret, dTurret, fTurret, lTurret);
                    turretTargetAngle = debugTargetAngle;
                }
                turretTargetAngle = Math.max(-82.5, Math.min(277.5, turretTargetAngle)); //Negative is ccw
                turretControl.setTarget(angleToTicks(turretTargetAngle));
                turretControl.update(turret.getCurrentPosition());
                turret.setPower(turretControl.runPDFL(angleToTicks(turretAngleTolerance)));
            }

            hoodTargetPosition = hoodLinreg == 0 ? (debugHoodTargetPosition) : (Math.max(0, Math.min(.9, hoodLinreg * RobotSubsystem.INSTANCE.getDistanceToGoalInches())));
            hood.setPosition(hoodTargetPosition);

        }
    }

    public void sendTelemetry(){
        ActiveOpMode.telemetry().addData("Hood Target: ", hoodTargetPosition);
        ActiveOpMode.telemetry().addData("Current Velo: ", getCurrentVelocityRPM());
        ActiveOpMode.telemetry().addData("Target Velo: ", toRPM(launcherControl.getGoal().getVelocity()));
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Current Velo: ", getCurrentVelocityRPM());
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Target Velo: ", toRPM(launcherControl.getGoal().getVelocity()));
        ActiveOpMode.telemetry().addData("Turret Target: ", turretTargetAngle);
        ActiveOpMode.telemetry().addData("Turret Debug Target: ", debugTargetAngle);
        ActiveOpMode.telemetry().addData("Turret Current: ", getCurrentAngle());
    }

    public double getTurretTarget(){
        return turretTargetAngle;
    }

    public void setHoodTarget(double angle){
        angle = Math.max(22, Math.min(45, angle));
        debugHoodTargetPosition = (angle - 20) / 25;
    }

    public double getHoodTarget(){
        return hoodTargetPosition;
    }

    //Turret Commands
    public Command SetTurretState(TurretState state){
        return new LambdaCommand()
                .setStart(() -> setTurretState(state))
                .setIsDone(OuttakeSubsystem::turretFinished)
                .requires("Turret");
    }

    public Command ScanPattern(){
        return new SequentialGroup(
                SetTurretState(TurretState.OBELISK),
                new WaitUntil(RobotSubsystem.INSTANCE::getPatternFull),
                SetTurretState(TurretState.GOAL)
        );
    }

    public Command SetFlywheelState(FlywheelState state){
        return new LambdaCommand()
                .setStart(() -> launcherState = state)
                .setIsDone(() -> true)
                .requires("Launcher");
    }

    public void setTurretTargetAngle(double angle){
        turretTargetAngle = angle;
    }

    //Math helpers

    public double toRPM(double velo){
        return velo * ((double) 60 / 28);
    }

    public double toTicksPerSec(double velo){
        return velo * ((double) 28 / 60);
    }

    public double getCurrentVelocityRPM() {
        return toRPM(launcherGroup.getVelocity());
    }
    public static boolean turretFinished(){
        return Math.abs(ticksToAngle(turret.getCurrentPosition()) - turretTargetAngle) < turretAngleTolerance;
    }
    public static void setTurretState(TurretState state){
        turretState = state;
    }
    public static TurretState getTurretState() {
        return turretState;
    }

    //Uses degrees
    public static double angleToTicks(double angle) {
        return angle * UniConstants.TURRET_TICKS_PER_DEGREE;
    }

    public static double ticksToAngle(double ticks) {
        return (ticks / UniConstants.TURRET_TICKS_PER_DEGREE) % 360;
    }

    public static double getHoodTargetPosition(){return hoodTargetPosition;}

    public void resetTurret(){
        turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getCurrentAngle(){
        return ticksToAngle(turret.getCurrentPosition());
    }

    public enum TurretState {
        FORWARD,
        OBELISK,
        GOAL
    }

    public enum FlywheelState {
        MEDIUM,
        FULL,
        OFF
    }

}
