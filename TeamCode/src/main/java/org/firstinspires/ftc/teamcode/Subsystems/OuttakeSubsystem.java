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
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

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
    public static  PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.0006566, 0, 0); //TODO: Tune
    private static ControlSystem launcherControl;
    public static double targetVeloRPM = 0;

    //Hood Stuffs
    private static final ServoEx hood = new ServoEx("HOOD");
    private static double hoodTargetPosition = .5;
    public static boolean hoodLinreg = true;
    public static double debugHoodTargetPosition = .75;

    //Turret Stuffs
    private static final MotorEx turret = new MotorEx(UniConstants.TURRET_STRING).zeroed().brakeMode();
    private static TurretState turretState = TurretState.FORWARD;
    public static double pTurret = 0.00024, dTurret = 0, lTurret = 0.098, fTurret = 0.02;
    private PDFLController turretControl = new PDFLController(pTurret, dTurret, fTurret, lTurret);
    private static double turretTargetAngle = 0;
    public static double turretAngleTolerance = .75;

    public static double debugTargetAngle = 0;

    public static double midRPM = 2750;
    public static double lazyRPM = 500;

    public static double kS = 0.09, kVShort = 0.00042;

    private static double oldTurret = 0;

    public static double userAdded = 0;

    @Override
    public void initialize() {
        launcherControl = ControlSystem.builder()
                .velPid(launcherPIDCoefficients)
                .basicFF(kVShort, 0, kS)
                .build();
        turretControl.setPDFL(pTurret, dTurret, fTurret, lTurret);
    }

    @Override
    public void periodic() {
        if (ActiveOpMode.isStarted()) {
            launcherControl = ControlSystem.builder()
                    .velPid(launcherPIDCoefficients)
                    .basicFF(kVShort, 0, kS)
                    .build();
            //Flywheel control
            if (!debug) {
                switch (launcherState) {
                    case OFF:
                        launcherControl.setGoal(new KineticState(0, toTicksPerSec(0)));
                        break;
                    case MEDIUM:
                        launcherControl.setGoal(new KineticState(0, toTicksPerSec(midRPM)));
                        break;
                    case LAZY:
                        launcherControl.setGoal(new KineticState(0, toTicksPerSec(lazyRPM)));
                        break;
                    case INTERPOLATED:
                        launcherControl.setGoal(new KineticState(0, toTicksPerSec(getInterpolatedVelo(RobotSubsystem.INSTANCE.getDistanceToGoalInches()))));
                        hoodLinreg = true;
                        break;
                }
                launcherGroup.setPower(12/RobotSubsystem.INSTANCE.getVoltage() * Math.max(0, Math.min(1, launcherControl.calculate(
                                new KineticState(launcherGroup.getCurrentPosition(), launcherGroup.getVelocity())
                        )))
                );
            } else {
                launcherGroup.setPower(debugPower);
            }

            //Turret control
            if (turretEnabled) {
                if(debug) {
                    turretControl.setPDFL(pTurret, dTurret, fTurret, lTurret);
                    turretTargetAngle = debugTargetAngle;
                }
                turretTargetAngle = Math.max(-50, Math.min(100, turretTargetAngle)); //Negative is ccw
                turretControl.setTarget(angleToTicks(turretTargetAngle));
                turretControl.update(getTurretPosition());
                turret.setPower((12/RobotSubsystem.INSTANCE.getVoltage() * turretControl.runPDFL(angleToTicks(turretAngleTolerance))));
            } else {
                turret.setPower(0);
            }

            hoodTargetPosition = !hoodLinreg ? (debugHoodTargetPosition) : (getInterpolatedHood(RobotSubsystem.INSTANCE.getDistanceToGoalInches()));
            hood.setPosition(hoodTargetPosition);

            oldTurret = getTurretPosition();

        }
    }

    public void sendTelemetry(){
        ActiveOpMode.telemetry().addData("Hood Target: ", hoodTargetPosition);
        ActiveOpMode.telemetry().addData("Current Velo: ", getCurrentVelocityRPM());
        ActiveOpMode.telemetry().addData("Current Velo L: ", toRPM(leftLaunchMotor.getVelocity()));
        ActiveOpMode.telemetry().addData("Current Velo R: ", toRPM(rightLaunchMotor.getVelocity()));
        ActiveOpMode.telemetry().addData("Target Velo: ", toRPM(launcherControl.getGoal().getVelocity()));
        ActiveOpMode.telemetry().addData("Turret Target: ", turretTargetAngle);
        ActiveOpMode.telemetry().addData("Turret Current: ", getCurrentAngle());
        ActiveOpMode.telemetry().addData("Turret Finished?: ", turretFinished());
    }

    public static Command addUserAdded(){
        return new InstantCommand(() -> userAdded += 100);
    }

    public static Command subUserAdded(){
        return new InstantCommand(() -> userAdded -= 100);
    }
    public void setTurretEnabled(boolean enabled){
        turretEnabled = enabled;
    }


    public double getTurretTarget(){
        return turretTargetAngle;
    }

    public double getInterpolatedVelo(double dist){
        return Math.max(0, Math.min(3250,
                -0.00208274 * Math.pow(dist, 3)
                + 0.518879 * Math.pow(dist, 2)
                - 24.00003 * dist
                + 2100 + userAdded));
    }

    public double getInterpolatedHood(double dist){
        return  Math.max(0, Math.min(1,
                0.00000116334 * Math.pow(dist, 3)
                - 0.000372395 * Math.pow(dist, 2)
                + 0.0422195 * dist
                - 0.872815));

    }

    public double getVelInterpolatedHood(double dist, double vel) {
        // TODO: Run new, better linregs to determine hood variance with velocity and such
        // TODO: could also maybe provide target height or airtime or whatever for better sorting and aim
        double normalAngle = -(2.74194 * Math.pow(10, -7)) * Math.pow(dist, 4) + 0.0000901357 * Math.pow(dist, 3) - 0.0106195 * Math.pow(dist, 2) + (0.535594 * dist) + 9.20171;
        return normalAngle + (getInterpolatedVelo(dist) - vel) * 0.001;
    }

    public void setHoodTarget(double angle){
        angle = Math.max(22, Math.min(45, angle));
        debugHoodTargetPosition = (angle - 20) / 25;
    }

    public void setHood(double targ){

        debugHoodTargetPosition = targ;
    }

    public double getHoodTarget(){
        return hoodTargetPosition;
    }

    //Turret Commands
    public double getTurretPosition(){
        return IntakeSubsystem.INSTANCE.active.getCurrentPosition();
    }

    public Command SetTurretState(TurretState state){
        return new LambdaCommand()
                .setStart(() -> setTurretState(state))
                .setIsDone(OuttakeSubsystem::turretFinished)
                .requires("Turret");
    }



    public Command ScanPattern(){
        return new LambdaCommand()
                .setStart(SetTurretState(TurretState.OBELISK))
                .setIsDone(RobotSubsystem::getPatternFull)
                .setStop((inter) -> SetTurretState(TurretState.GOAL));
    }

    public Command SetFlywheelState(FlywheelState state){
        return new LambdaCommand()
                .setStart(() -> launcherState = state)
                .setIsDone(OuttakeSubsystem::flywheelGood)
                .requires("Launcher");
    }

    public static boolean flywheelGood(){
        return Math.abs(Math.abs(OuttakeSubsystem.INSTANCE.getCurrentVelocityRPM()) - Math.abs(targetVeloRPM)) < 150;
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
        return Math.abs(Math.abs(ticksToAngle(OuttakeSubsystem.INSTANCE.getTurretPosition())) - Math.abs(turretTargetAngle)) < turretAngleTolerance &&
                Math.abs(Math.abs(ticksToAngle(OuttakeSubsystem.INSTANCE.getTurretPosition())) - Math.abs(ticksToAngle(oldTurret))) < 5;
    }
    public static void setTurretState(TurretState state){
        turretState = state;
    }
    public static TurretState getTurretState() {
        return turretState;
    }

    //Uses degrees
    public static double angleToTicks(double angle) {
        return angle * UniConstants.ENCODER_TICKS_PER_DEGREE;
    }

    public static double ticksToAngle(double ticks) {
        return (ticks / UniConstants.ENCODER_TICKS_PER_DEGREE) % 360;
    }

    public static double getHoodTargetPosition(){return hoodTargetPosition;}

    public void resetTurret(){
        IntakeSubsystem.INSTANCE.active.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeSubsystem.INSTANCE.active.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getCurrentAngle(){
        return ticksToAngle(getTurretPosition());
    }

    public enum TurretState {
        FORWARD,
        OBELISK,
        GOAL
    }

    public enum FlywheelState {
        MEDIUM,
        LAZY,
        OFF,
        INTERPOLATED
    }

}
