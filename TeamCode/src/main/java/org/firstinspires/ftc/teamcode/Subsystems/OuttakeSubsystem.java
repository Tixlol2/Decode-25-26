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
    public static  PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.00045, 0, 0);
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
    public static double pTurret = 0.0004, dTurret = 0, lTurret = 0.12, fTurret = 0.0;
    private PDFLController turretControl = new PDFLController(pTurret, dTurret, fTurret, lTurret);
    private static double turretTargetAngle = 0;
    public static double turretAngleTolerance = .75;

    public static double debugTargetAngle = 0;

    public static double midRPM = 2750;
    public static double lazyRPM = 500;

    public static double kS = 0.09, kVShort = 0.00042;

    private static double oldTurret = 0;

    public static double userAdded = 0;

    // ── Shooter Geometry ─────────────────────────────────────────────────────
    // All distances in meters. Measure from floor to center of launch point / hoop.
    public static double LAUNCHER_HEIGHT_M   = 0.3175;  // TODO: measure
    public static double HOOP_HEIGHT_M       = 1.1;  // regulation hoop height
    public static double WHEEL_RADIUS_M      = 0.092;  // TODO: measure

    // Servo angle range: servoMinDeg → 0.0, servoMaxDeg → 1.0
    public static double HOOD_SERVO_MIN_DEG  = 20.0;
    public static double HOOD_SERVO_MAX_DEG  = 45.0;

    // Single tunable correction (degrees) applied to physics-solved angle.
    // Positive = aim higher, negative = aim lower. Tune empirically.
    public static double hoodPhysicsCorrection = 0.0;

    // Prefer high arc (true) or low arc (false) solution from physics solver.
    public static boolean preferHighArc = false;

    private static final double G             = 9.81;
    private static final double INCHES_TO_M   = 0.0254;

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
                        launcherControl.setGoal(new KineticState(0, toTicksPerSec(getTargetVelocityRPM(RobotSubsystem.INSTANCE.getDistanceToGoalInches()))));
                        hoodLinreg = true;
                        break;
                    case REACTIVE:
                        launcherControl.setGoal(new KineticState(0, toTicksPerSec(getTargetVelocityRPM(RobotSubsystem.INSTANCE.getDistanceToGoalInches()))));
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
                turretTargetAngle = Math.max(-50, Math.min(180, turretTargetAngle)); //Negative is ccw
                turretControl.setTarget(angleToTicks(turretTargetAngle));
                turretControl.update(getTurretPosition());
                turret.setPower(Math.min(.6,(12/RobotSubsystem.INSTANCE.getVoltage() * turretControl.runPDFL(angleToTicks(turretAngleTolerance)))));
            } else {
                turret.setPower(0);
            }

            // Hood: use physics-coupled solver in INTERPOLATED state (reacts to actual flywheel vel),
            // fall back to debug position otherwise.
            if (!hoodLinreg) {
                hoodTargetPosition = debugHoodTargetPosition;
            } else if (launcherState == FlywheelState.INTERPOLATED) {
                hoodTargetPosition = getInterpolatedHood(
                        RobotSubsystem.INSTANCE.getDistanceToGoalInches()
                );
            } else if (launcherState == FlywheelState.REACTIVE){
                hoodTargetPosition = getTargetHoodPosition(RobotSubsystem.INSTANCE.getDistanceToGoalInches(), getCurrentVelocityRPM());
            }
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

    /**
     * Returns the target flywheel velocity in RPM for a given distance.
     * Purely distance-driven — feed this into your ControlSystem goal.
     * userAdded allows driver trim at runtime.
     */
    public double getTargetVelocityRPM(double distInches) {
        return Math.max(0, Math.min(3250,
                -0.00188274 * Math.pow(distInches, 3)
                        + 0.568879   * Math.pow(distInches, 2)
                        - 24   * distInches
                        + 2175 + userAdded));
    }

    /**
     * Returns the target hood servo position (0.0–1.0) for a given distance
     * and the *actual* flywheel velocity at the moment of firing.
     *
     * Uses a physics solver (inverted projectile equations) so that velocity
     * deviations — including per-shot flywheel bleed in a burst — automatically
     * produce a corrected angle rather than being ignored.
     *
     * hoodPhysicsCorrection (degrees, @Configurable) absorbs real-world offsets
     * like air resistance and launcher geometry. Tune until shot 1 lands on target,
     * then verify shots 2 and 3 converge without further adjustment.
     *
     * Falls back to the distance-only regression if no physics solution exists
     * (e.g. velocity too low to reach the target at this distance).
     *
     * @param distInches   distance to goal from follower, in inches
     * @param actualVelRPM current flywheel RPM from encoder (getCurrentVelocityRPM())
     */
    public double getTargetHoodPosition(double distInches, double actualVelRPM) {
        double distM   = distInches * INCHES_TO_M;
        double muzzleV = rpmToMuzzleVelocity(actualVelRPM);
        double dy      = HOOP_HEIGHT_M - LAUNCHER_HEIGHT_M;

        double angleDeg = solveAngleDeg(muzzleV, distM, dy+.05);

        if (Double.isNaN(angleDeg)) {
            // Physics has no solution (vel too low) — fall back to distance-only regression
            return getInterpolatedHood(distInches);
        }

        double corrected = angleDeg + hoodPhysicsCorrection;
        return 1-angleDegToServo(corrected);
    }

    // ── Physics solver helpers ────────────────────────────────────────────────

    /**
     * Converts flywheel RPM to ball muzzle velocity in m/s.
     * Assumes ball exits at wheel surface speed (no slip).
     */
    private double rpmToMuzzleVelocity(double rpm) {
        return rpm * (2.0 * Math.PI * WHEEL_RADIUS_M) / 60.0;
    }

    /**
     * Inverted projectile equation: given launch speed, horizontal distance,
     * and vertical rise, returns the launch angle in degrees.
     *
     * Two solutions exist (low arc and high arc). preferHighArc selects which.
     * Returns NaN if no real solution exists (velocity insufficient).
     *
     * Derivation:
     *   y = x*tan(θ) - (g*x²)/(2v²cos²θ)
     *   Substituting sec²θ = 1 + tan²θ gives a quadratic in tan(θ):
     *   (gx²/2v²)*tan²θ - x*tanθ + (dy + gx²/2v²) = 0
     */
    private double solveAngleDeg(double v, double dx, double dy) {
        if (v < 0.01 || dx < 0.01) return Double.NaN;
        double v2 = v * v;
        double a  = (G * dx * dx) / (2.0 * v2);
        double b  = -dx;
        double c  = dy + a;
        double disc = b * b - 4.0 * a * c;
        if (disc < 0) return Double.NaN;

        double sqrtDisc = Math.sqrt(disc);
        double tanLow   = (-b - sqrtDisc) / (2.0 * a);
        double tanHigh  = (-b + sqrtDisc) / (2.0 * a);

        double chosen = preferHighArc ? tanHigh : tanLow;
        double angleRad = Math.atan(chosen);
        if (angleRad <= 0 || angleRad >= Math.PI / 2.0) return Double.NaN;
        return Math.toDegrees(angleRad);
    }

    /**
     * Maps a launch angle in degrees to a normalized servo position [0.0, 1.0]
     * using the configured hood servo range.
     */
    private double angleDegToServo(double angleDeg) {
        double pos = (angleDeg - HOOD_SERVO_MIN_DEG) / (HOOD_SERVO_MAX_DEG - HOOD_SERVO_MIN_DEG);
        return Math.max(0.0, Math.min(1.0, pos));
    }

    // ── Legacy regression (kept for fallback and backwards compat) ────────────

    /** @deprecated Use getTargetVelocityRPM(dist) instead. */
    @Deprecated
    public double getInterpolatedVelo(double dist) {
        return getTargetVelocityRPM(dist);
    }

    /** @deprecated Use getTargetHoodPosition(dist, actualVelRPM) instead. */
    @Deprecated
    public double getInterpolatedHood(double dist) {
        return Math.max(0, Math.min(1,
                0.00000116334 * Math.pow(dist, 3)
                        - 0.000372395  * Math.pow(dist, 2)
                        + 0.0422195    * dist
                        - 0.872815));
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
        INTERPOLATED,
        REACTIVE
    }

}