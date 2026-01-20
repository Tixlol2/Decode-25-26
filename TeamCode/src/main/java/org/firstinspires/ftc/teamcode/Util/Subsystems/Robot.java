package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.Subsystems.Slots.Slot;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class Robot extends SubsystemGroup {

    public static final Robot INSTANCE = new Robot();

    public static ArrayList<Slot.SlotState> pattern = new ArrayList<>(Arrays.asList(null, null, null));
    public static Supplier<ArrayList<Slot.SlotState>> patternSupplier;
    public static boolean patternFull = false;

    public static teamColor color = teamColor.BLUE;

    public static loggingState logstate = Robot.loggingState.ENABLED;
    public static boolean inTeleop = true;
    public static boolean automatedDrive = false;
    public static Timer shotTimer = new Timer();
    public static ArrayList<Slot> order;
    public static double standardWaitTime = .75;
    public static Pose previousPose = new Pose();
    ElapsedTime loopTimer = new ElapsedTime();
    private double distanceToGoal = 0;
    private JoinedTelemetry joinedTelemetry;

    private Robot() {
        super(
                MecDriveSubsystem.INSTANCE,
                TurretSubsystem.INSTANCE,
                IntakeSubsystem.INSTANCE,
                VisionSubsystem.INSTANCE,
                SlotsSubsystem.INSTANCE
        );
    }


    @Override
    public void initialize() {
        joinedTelemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        setGlobals();
        patternSupplier = () -> pattern;
    }

    @Override
    public void periodic() {

        loopTimer.reset();

        distanceToGoal = MecDriveSubsystem.INSTANCE.getDistanceToGoal();
        //Handles turret aiming
        switch (TurretSubsystem.state) {
            case FORWARD:
                TurretSubsystem.INSTANCE.setTargetAngle(0);
                break;
            case GOAL:
                TurretSubsystem.INSTANCE.setTargetAngle(-MecDriveSubsystem.INSTANCE.getGoalAngle());
                break;
            case OBELISK:
                TurretSubsystem.INSTANCE.setTargetAngle(-MecDriveSubsystem.INSTANCE.getObeliskAngle());
                break;
        }


        //Handles pattern updating
        if (pattern.contains(null)) {
            pattern = VisionSubsystem.INSTANCE.getPattern();
            patternFull = !pattern.contains(null);
        }


        joinedTelemetry.addData("Loop Time (ms) ", loopTimer.milliseconds());
        joinedTelemetry.addData("Pattern: ", pattern);
        joinedTelemetry.addData("Pattern Full: ", patternFull);


        joinedTelemetry.update();

    }


    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public JoinedTelemetry getJoinedTelemetry() {
        return joinedTelemetry;
    }


    public Command FaceGoal() {
        return new ParallelGroup(
                TurretSubsystem.INSTANCE.TurretGoal(),
                new TurnTo(color == teamColor.BLUE ? Angle.fromDeg(UniConstants.ANGLE_BLUE_GOAL_DEGREES) : Angle.fromDeg(UniConstants.ANGLE_RED_GOAL_DEGREES))
        );
    }

    public Command Park() {
        return new ParallelGroup(
                StopSubsystems(),
                new FollowPath(MecDriveSubsystem.INSTANCE.createParkPath(), true)
        );
    }

    public Command StopSubsystems() {
        return new ParallelGroup(
                TurretSubsystem.INSTANCE.TurretForward(),
                IntakeSubsystem.INSTANCE.stopActive(),
                TurretSubsystem.INSTANCE.SetFlywheelState(TurretSubsystem.FlywheelState.OFF)
        );
    }

    public void setGlobalTelemetry() {
        MecDriveSubsystem.INSTANCE.setTelemetry(joinedTelemetry);
        TurretSubsystem.INSTANCE.setTelemetry(joinedTelemetry);
        IntakeSubsystem.INSTANCE.setTelemetry(joinedTelemetry);
        VisionSubsystem.INSTANCE.setTelemetry(joinedTelemetry);

        for (Slot slot : SlotsSubsystem.INSTANCE.slots) {
            slot.setTelemetry(joinedTelemetry);
        }
    }

    public void setGlobalColor() {
        MecDriveSubsystem.INSTANCE.setColor(color);
        VisionSubsystem.INSTANCE.setColor(color);

    }

    public void setGlobals() {
        setGlobalTelemetry();
        setGlobalColor();
    }


    public boolean withinRange(double value, double target, double range) {
        return Math.abs(value - target) < range;
    }


    public enum teamColor {
        RED,
        BLUE
    }

    //Enums
    public enum loggingState {
        DISABLED,
        ENABLED,
        EXTREME
    }
}


