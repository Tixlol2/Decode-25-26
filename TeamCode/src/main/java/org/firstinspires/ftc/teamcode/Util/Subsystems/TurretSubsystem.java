package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.Lerp;
import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.util.HashMap;
import java.util.Map;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class TurretSubsystem implements Subsystem {
    public static final TurretSubsystem INSTANCE = new TurretSubsystem();
    public static final HashMap<FlywheelState, Double> flywheelSpeedRPM = new HashMap<>(
            Map.of(FlywheelState.SHORT, 2600.0, FlywheelState.FAR, 3000.0, FlywheelState.OFF, 0.0)
    );
    public static double targetVelocity = 0;
    public static double pLaunch = 0, iLaunch = 0, dLaunch = 0;
    public static double turretTargetAngle = 0;
    public static double pTurret = 0.003, dTurret = 0, lTurret = .1, fTurret = 0;
    public static boolean debug = false;
    public static double debugPower = 0;
    public static TurretState state = TurretState.FORWARD;

    private final PDFLController turretControl = new PDFLController(pTurret, dTurret, fTurret, lTurret);
    public static ControlSystem controller;
    public Lerp lerp = new Lerp(0, 0, 0);
    // put hardware, commands, etc here
    JoinedTelemetry telemetry;

    MotorEx launcherBottomWithEncoder = new MotorEx(UniConstants.LAUNCHER_BOTTOM_STRING).floatMode();
    MotorEx launcherTop = new MotorEx(UniConstants.LAUNCHER_TOP_STRING).floatMode();
    MotorGroup launcherGroup = new MotorGroup(launcherBottomWithEncoder, launcherTop);

    MotorEx turret = new MotorEx(UniConstants.TURRET_STRING).floatMode().zeroed().brakeMode();
    private double launcherCurrentVelo = 0;

    private double turretCurrentPos = 0;
    private FlywheelState FWState = FlywheelState.OFF;
    public static PIDCoefficients coefficients = new PIDCoefficients(pLaunch, iLaunch, dLaunch);

    @Override
    public void initialize() {
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        FWState = FlywheelState.OFF;
        turretControl.setPDFL(pTurret, dTurret, fTurret, lTurret);
        controller = ControlSystem.builder()
                .velPid(coefficients)
                .build();

    }

    @Override
    public void periodic() {
        if (!ActiveOpMode.opModeInInit()) {
            // Launcher control (this looks fine)
            if(!debug) {
                targetVelocity = getTargetFromMap();
                controller.setGoal(new KineticState(0, targetVelocity));
                launcherGroup.setPower(controller.calculate(new KineticState(0, getCurrentVelocityRPM())));
            } else {launcherGroup.setPower(debugPower);}

            //Full turret control
            turretCurrentPos = turret.getCurrentPosition();
            turretTargetAngle = Math.max(-180, Math.min(35, turretTargetAngle)); //Negative is ccw
            turretControl.setTarget(angleToTicks(turretTargetAngle));
            turretControl.update(turretCurrentPos);
            turret.setPower(turretControl.runPDFL(angleToTicks(0.5)));
        }
    }


    public FlywheelState getFlywheelState() {
        return FWState;
    }

    public void setFlywheelState(FlywheelState state) {
        FWState = state;
    }

    public double getTargetFromMap() {
        return (flywheelSpeedRPM.get(FWState));
    }

    public Command SetFlywheelState(FlywheelState state) {
        return new SequentialGroup(
                new LambdaCommand()
                        .setStart(() -> setFlywheelState(state))
                        .setIsDone(() -> controller.isWithinTolerance(new KineticState(0, 200))));
        //new IfElseCommand(() -> state != FlywheelState.OFF, new Delay(2)));
    }

    public boolean veloWithinRangeBool() {
        return (Math.abs(targetVelocity - launcherCurrentVelo) < 200);
    }

    public void setTurretState(TurretState state) {
        TurretSubsystem.state = state;
    }

    public void setTargetAngle(double angleDeg) {
        turretTargetAngle = angleDeg;
    }

    //Uses degrees
    public double angleToTicks(double angle) {
        return angle * UniConstants.TURRET_TICKS_PER_DEGREE;
    }

    //Uses degrees
    public double ticksToAngle(double ticks) {
        return (ticks / UniConstants.TURRET_TICKS_PER_DEGREE) % 360;
    }

    public double getCurrentVelocityRPM() {
        return -(launcherGroup.getVelocity() * 60 / 28);
    }

    public double getCurrentAngle() {
        return ticksToAngle(turretCurrentPos);
    }

    public boolean turretFinished() {
        return Robot.INSTANCE.withinRange(getCurrentAngle(), getTurretTargetAngle(), 1);
    }

    public void init() {
        turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretCurrentPos = 0;
        turretTargetAngle = 0;
        targetVelocity = 0;
        controller.setGoal(new KineticState(0.0));
    }


    public double getTurretTargetAngle() {
        return turretTargetAngle;
    }

    public Command TurretForward() {
        return new LambdaCommand()
                .setStart(() -> {
                    TurretSubsystem.INSTANCE.setTurretState(TurretSubsystem.TurretState.FORWARD);
                })
                .setIsDone(TurretSubsystem.INSTANCE::turretFinished);
    }

    public Command TurretGoal() {
        return new LambdaCommand()
                .setStart(() -> {
                    TurretSubsystem.INSTANCE.setTurretState(TurretSubsystem.TurretState.GOAL);
                })
                .setIsDone(TurretSubsystem.INSTANCE::turretFinished);
    }

    public Command TurretObelisk() {
        return new LambdaCommand()
                .setStart(() -> {
                    TurretSubsystem.INSTANCE.setTurretState(TurretSubsystem.TurretState.OBELISK);
                })
                .setIsDone(TurretSubsystem.INSTANCE::turretFinished);
    }

    public Command ScanPattern(){
        return new LambdaCommand()
                .setStart(() -> {
                    TurretSubsystem.INSTANCE.setTurretState(TurretSubsystem.TurretState.OBELISK);
                })
                .setIsDone(() -> Robot.patternFull)
                .setStop((interrupted) -> {TurretSubsystem.INSTANCE.setTurretState(TurretState.GOAL);});
    }

    public void sendTelemetry(Robot.loggingState state) {
        switch (state) {
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF OUTTAKE LOG");
                telemetry.addData("Target RPM ", targetVelocity);
                telemetry.addData("Current RPM ", (getCurrentVelocityRPM()));
                telemetry.addLine();
                telemetry.addData("Turret Position Deg ", ticksToAngle(turretCurrentPos));
                telemetry.addData("Turret Target Deg ", turretTargetAngle);
                telemetry.addLine("END OF OUTTAKE LOG");
                break;
            case EXTREME:

                telemetry.addLine("START OF OUTTAKE LOG");

                telemetry.addLine("END OF OUTTAKE LOG");
        }
    }

    public void setTelemetry(JoinedTelemetry telemetry) {
        this.telemetry = telemetry;
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
