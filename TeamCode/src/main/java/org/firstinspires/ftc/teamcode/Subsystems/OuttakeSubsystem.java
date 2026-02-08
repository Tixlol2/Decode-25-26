package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class OuttakeSubsystem implements Subsystem {

    public static final OuttakeSubsystem INSTANCE = new OuttakeSubsystem();

    //Misc. Stuffs
    public static boolean debug = false;
    public static double debugPower = 0;
    public static boolean turretEnabled = false;

    //Launcher Stuffs
    private static final MotorEx leftLaunchMotor = new MotorEx(UniConstants.LAUNCHER_LEFT_STRING).floatMode().reversed();
    private static final MotorEx rightLaunchMotor = new MotorEx(UniConstants.LAUNCHER_RIGHT_STRING).floatMode();
    private static final MotorGroup launcherGroup = new MotorGroup(rightLaunchMotor, leftLaunchMotor); //Bottom has encoder, put first
    private static FlywheelState launcherState = FlywheelState.OFF;
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.00, 0, 0); //TODO: Tune
    private static ControlSystem launcherControl;
    public static double targetVeloRPM = 0;

    //Hood Stuffs
    private static final ServoEx hood = new ServoEx(UniConstants.HOOD_STRING);
    private static double hoodTargetPosition = .5;
    public static double hoodLinreg = 0;
    public static double debugHoodTargetPosition = .5;

    //Turret Stuffs
    private static final MotorEx turret = new MotorEx(UniConstants.TURRET_STRING).zeroed().brakeMode();
    private static TurretState turretState = TurretState.FORWARD;
    public static double pTurret = 0.000, dTurret = 0, lTurret = 0.0, fTurret = 0; //TODO: Tune
    private final PDFLController turretControl = new PDFLController(pTurret, dTurret, fTurret, lTurret);
    private static double turretTargetAngle = 0;
    public static double turretAngleTolerance = .5;

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
            switch(launcherState) {
                case OFF:
                    launcherGroup.setPower(0);
                    break;
                case MEDIUM:
                    launcherGroup.setPower(.5);
                    break;
                case FULL:
                    launcherGroup.setPower(1);
                    break;
            }
        } else {
            launcherGroup.setPower(debugPower);
        }

        //Turret control
        if(turretEnabled) {
            turretTargetAngle = Math.max(-180, Math.min(35, turretTargetAngle)); //Negative is ccw
            turretControl.setTarget(angleToTicks(turretTargetAngle));
            turretControl.update(turret.getCurrentPosition());
            turret.setPower(turretControl.runPDFL(angleToTicks(turretAngleTolerance)));
        }

        hoodTargetPosition = debug ? (debugHoodTargetPosition) : (Math.max(0, Math.min(1, hoodLinreg * RobotSubsystem.INSTANCE.getDistanceToGoalInches())));
        hood.setPosition(hoodTargetPosition);

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
    public double getCurrentVelocityRPM() {
        return (launcherGroup.getVelocity() * 60 / 28);
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
