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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class TurretSubsystem implements Subsystem {
    public static final TurretSubsystem INSTANCE = new TurretSubsystem();
    public static int targetVelocity = 0;
    public static double pLaunch = .00, dLaunch = 0, fLaunch = 0, lLaunch = 0.01;
    public static double turretTargetAngle = 0;
    public static double pTurret = 0.0025, dTurret = 0, lTurret = 0.125, fTurret = 0;
    public static boolean debug = false;
    public static double debugPower = 0;
    public static TurretState state = TurretState.FORWARD;
    public static boolean launcherPDFL = false;
    public static double veloLinReg = 0.0000001;
    public static double posLinReg = 0.0000001;
    public static HashMap<FlywheelState, Integer> flywheelSpeedsRPM = new HashMap<>(
            Map.of(FlywheelState.SHORT, 2500, FlywheelState.FAR, 4000, FlywheelState.OFF, 0)
    );
    private final PDFLController turretControl = new PDFLController(pTurret, dTurret, fTurret, lTurret);
    private final PDFLController launcherController = new PDFLController(pLaunch, dLaunch, fLaunch, lLaunch);
    public Lerp lerp = new Lerp(0, 0, 0);
    // put hardware, commands, etc here
    JoinedTelemetry telemetry;

    MotorEx launcherBottomWithEncoder = new MotorEx(UniConstants.LAUNCHER_BOTTOM_STRING).floatMode();
    MotorEx launcherTop = new MotorEx(UniConstants.LAUNCHER_TOP_STRING).floatMode();
    MotorGroup launcherGroup = new MotorGroup(launcherBottomWithEncoder, launcherTop);

    MotorEx turret = new MotorEx(UniConstants.TURRET_STRING).floatMode().zeroed().brakeMode();
    private double launcherCurrentVelo = 0;
    private double power = 0;
    private double turretCurrentPos = 0;
    private FlywheelState FWState = FlywheelState.OFF;

    @Override
    public void initialize() {
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        FWState = FlywheelState.OFF;
    }

    @Override
    public void periodic() {
        if (!ActiveOpMode.opModeInInit()) {
            // Launcher control (this looks fine)
            if (!launcherPDFL) {
                new SetPower(launcherGroup, debugPower).run();
            } else {
                if (!debug) {
                    if (FWState != FlywheelState.INTERPOLATED) {
                        targetVelocity = getTargetFromMap();
                    } else {
                        targetVelocity = (int) (posLinReg * Robot.INSTANCE.getDistanceToGoal()); //TODO: Do pos vs velo linreg
                    }
                }

                launcherController.setPDFL(pLaunch, dLaunch, fLaunch, lLaunch);

                launcherCurrentVelo = getCurrentVelocityRPM();
                launcherController.setTarget(targetVelocity);
                launcherController.update(launcherCurrentVelo);
                power += lerp.constantLerp(power, targetVelocity * veloLinReg, 1);//Math.min(pdfl.runPDFL(20),.1); //TODO: Do own linreg
                // clamp power to valid motor range
                if (Double.isNaN(power)) {
                    power = 0;
                }
                power = Math.max(0.0, Math.min(1.0, power));
                new SetPower(launcherGroup, power + launcherController.runPDFL(10)).run();
            }

            //Full turret control
            //turretControl.setPDFL(pTurret, dTurret, fTurret, lTurret);
            turretCurrentPos = turret.getCurrentPosition();
            turretTargetAngle = Math.max(-265, Math.min(45, turretTargetAngle)); //Negative is ccw
            turretControl.setTarget(angleToTicks(turretTargetAngle));
            turretControl.update(turretCurrentPos);
            new SetPower(turret, turretControl.runPDFL(angleToTicks(0.5))).run();
        }
    }


    public void setTargetVelocityTicks(int velo) {
        targetVelocity = velo;
    }

    public FlywheelState getFlywheelState() {
        return FWState;
    }

    public void setFlywheelState(FlywheelState state) {
        FWState = state;
    }

    public int getTargetFromMap() {
        return (int) (flywheelSpeedsRPM.get(FWState) / 2.1);
    }

    public Command SetFlywheelState(FlywheelState state) {
        return new LambdaCommand()
                .setStart(() -> setFlywheelState(state))
                .setIsDone(this::veloWithinRangeBool);
    }

    public Command SetTargetVelo(int velo, boolean usingRPM) {
        return new LambdaCommand()
                .setStart(() -> {
                    if (usingRPM) {
                        setTargetVelocityRPM(velo);
                    } else {
                        setTargetVelocityTicks(velo);
                    }
                })
                .setIsDone(this::veloWithinRangeBool);
    }

    public Command SetTargetVelo(int velo) {
        return new LambdaCommand()
                .setStart(() -> {
                    setTargetVelocityRPM(velo);
                })
                .setIsDone(this::veloWithinRangeBool);
    }

    public boolean veloWithinRangeBool() {
        return (Math.abs(targetVelocity - launcherCurrentVelo) < 200);
    }

    public void setTargetVelocityRPM(int velo) {
        targetVelocity = velo;
    }

    public void setMotorPower(double power) {
        debugPower = Math.max(-.7, Math.min(1, power));
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
        return -(launcherBottomWithEncoder.getVelocity() * 60 / 28);
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
        power = 0;
    }


    public double getTurretTargetAngle() {
        return turretTargetAngle;
    }

    public Command SetMotorPower(double power) {
        return new InstantCommand(() -> setMotorPower(power));
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

    public void sendTelemetry(Robot.loggingState state) {
        switch (state) {
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF OUTTAKE LOG");
                telemetry.addData("Current RPM ", (getCurrentVelocityRPM()));
                telemetry.addData("Power: ", power);
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
        INTERPOLATED,
        OFF
    }


}
