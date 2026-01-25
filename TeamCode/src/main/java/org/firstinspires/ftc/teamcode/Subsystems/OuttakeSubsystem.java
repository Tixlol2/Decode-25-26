package org.firstinspires.ftc.teamcode.Subsystems;

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
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

public class OuttakeSubsystem implements Subsystem {

    public static final OuttakeSubsystem INSTANCE = new OuttakeSubsystem();

    //Misc. Stuffs
    public static boolean debug = false;
    public static double debugPower = 0;

    //Launcher Stuffs
    private static final MotorEx launcherTop = new MotorEx(UniConstants.LAUNCHER_TOP_STRING).floatMode();
    private static final MotorEx launcherBottom = new MotorEx(UniConstants.LAUNCHER_BOTTOM_STRING).floatMode();
    private static final MotorGroup launcherGroup = new MotorGroup(launcherBottom, launcherTop); //Bottom has encoder, put first
    private static FlywheelState launcherState = FlywheelState.OFF;
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.00, 0, 0);
    private static ControlSystem launcherControl;
    public static double targetVeloRPM = 0;



    //Turret Stuffs
    private static final MotorEx turret = new MotorEx(UniConstants.TURRET_STRING).zeroed().brakeMode();
    private static TurretState turretState = TurretState.FORWARD;
    public static double pTurret = 0.003, dTurret = 0, lTurret = .1, fTurret = 0;
    private final PDFLController turretControl = new PDFLController(pTurret, dTurret, fTurret, lTurret);
    private static double turretTargetAngle = 0;
    public static double angleTolerance = .5;

    @Override
    public void initialize() {
        launcherControl = ControlSystem.builder()
                .velPid(launcherPIDCoefficients)
                .build();
    }

    @Override
    public void periodic() {
        //Flywheel control
        if(!debug) {
            launcherControl.setGoal(new KineticState(0, targetVeloRPM));
            launcherGroup.setPower(launcherControl.calculate(new KineticState(0, getCurrentVelocityRPM())));
        } else {
            launcherGroup.setPower(debugPower);
        }

        //Turret control
        turretTargetAngle = Math.max(-180, Math.min(35, turretTargetAngle)); //Negative is ccw
        turretControl.setTarget(angleToTicks(turretTargetAngle));
        turretControl.update(turret.getCurrentPosition());
        turret.setPower(turretControl.runPDFL(angleToTicks(0.5)));
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

    public void setTurretTargetAngle(double angle){
        turretTargetAngle = angle;
    }

    //Math helpers
    public double getCurrentVelocityRPM() {
        return -(launcherGroup.getVelocity() * 60 / 28);
    }
    public static boolean turretFinished(){
        return Math.abs(ticksToAngle(turret.getCurrentPosition()) - turretTargetAngle) < angleTolerance;
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


    public enum TurretState {
        FORWARD,
        OBELISK,
        GOAL
    }

    public enum FlywheelState {
        SHORT,
        FAR,
        OFF
    }

}
