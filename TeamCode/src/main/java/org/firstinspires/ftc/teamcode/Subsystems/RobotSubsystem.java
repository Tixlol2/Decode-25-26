package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Slots.BackSlot;
import org.firstinspires.ftc.teamcode.Subsystems.Slots.LeftSlot;
import org.firstinspires.ftc.teamcode.Subsystems.Slots.MainSlot;
import org.firstinspires.ftc.teamcode.Subsystems.Slots.RightSlot;
import org.firstinspires.ftc.teamcode.Util.Poses;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class RobotSubsystem extends SubsystemGroup {

    public static final RobotSubsystem INSTANCE = new RobotSubsystem();
    
    
    private RobotSubsystem(){
        super(
                OuttakeSubsystem.INSTANCE,
                VisionSubsystem.INSTANCE,
                IntakeSubsystem.INSTANCE,
                BackSlot.INSTANCE,
                LeftSlot.INSTANCE,
                RightSlot.INSTANCE
        );
    }

    public static ArrayList<MainSlot.SlotState> pattern = new ArrayList<>(Arrays.asList(null, null, null));
    public static Supplier<ArrayList<MainSlot.SlotState>> patternSupplier;
    public static boolean patternFull = false;

    public static Pose previousPose = new Pose();
    ElapsedTime loopTimer = new ElapsedTime();
    public static AllianceColor allianceColor = AllianceColor.BLUE;


    private double distanceToGoal = 0;
    private double goalAngle = 0;
    private double obeliskAngle = 0;

    private static Supplier<ArrayList<MainSlot>> result;
    private static final ArrayList<MainSlot> slots = new ArrayList<>(Arrays.asList(BackSlot.INSTANCE, LeftSlot.INSTANCE, RightSlot.INSTANCE));

    @Override
    public void initialize() {
        patternSupplier = () -> pattern;
        result = () -> determineOrder(patternSupplier.get());
    }


    @Override
    public void periodic() {

        loopTimer.reset();

        //Really consider updating this differently
        updateDistanceAndAngle();


        //Handles turret aiming
        switch (OuttakeSubsystem.getTurretState()) {
            case FORWARD:
                OuttakeSubsystem.INSTANCE.setTurretTargetAngle(0);
                break;
            case GOAL:
                OuttakeSubsystem.INSTANCE.setTurretTargetAngle(goalAngle);
                break;
            case OBELISK:
                OuttakeSubsystem.INSTANCE.setTurretTargetAngle(obeliskAngle);
                break;
        }


        //Handles pattern updating
        if (pattern.contains(null)) {
            pattern = VisionSubsystem.INSTANCE.getPattern();
            patternFull = !pattern.contains(null);
        }

        ActiveOpMode.telemetry().addData("Loop Times (ms) ", loopTimer.milliseconds());
        ActiveOpMode.telemetry().update();



    }

    public double getDistanceToGoal(){
        return distanceToGoal;
    }

    public boolean getPatternFull(){
        return patternFull;
    }

    public void updateDistanceAndAngle() {
        double x = 0, y = 0, obX = 0, obY = 0;

        switch (allianceColor) {
            case BLUE:
                x = Poses.blueGoal.getX() - PedroComponent.follower().getPose().getX();
                y = Poses.blueGoal.getY() - PedroComponent.follower().getPose().getY();
                break;
            case RED:
                x = Poses.redGoal.getX() - PedroComponent.follower().getPose().getX();
                y = Poses.redGoal.getY() - PedroComponent.follower().getPose().getY();
                break;
        }

        obX = Poses.obelisk.getX() - PedroComponent.follower().getPose().getX();
        obY = Poses.obelisk.getY() - PedroComponent.follower().getPose().getY();

        // In Pedro Pathing: +Y is forward (90), +X is right (0)
        // atan2(y, x) measures angle from +X axis
        // We want angle where +Y is 0 (forward for the robot)
        // So: fieldAngle = atan2(y, x) gives us angle from +X
        // To get angle from +Y: subtract 90 (or add 270, same thing)
        double fieldAngleToTarget = Math.toDegrees(Math.atan2(y, x)) - 90;
        double targetObelisk = Math.toDegrees(Math.atan2(obY, obX)) - 90;

        // Get robot heading in degrees (0 = facing +X, 90 = facing +Y)
        double robotHeading = (Math.toDegrees(PedroComponent.follower().getPose().getHeading()) - 90);

        // Calculate turret angle relative to robot
        goalAngle = (fieldAngleToTarget - robotHeading);
        obeliskAngle = (targetObelisk - robotHeading);

        // Clamp to -180 to 180
        while (goalAngle > 180) goalAngle -= 360;
        while (goalAngle < -180) goalAngle += 360;

        while (obeliskAngle > 180) obeliskAngle -= 360;
        while (obeliskAngle < -180) obeliskAngle += 360;

        //For some reason it works when they are negative????
        goalAngle *= -1;
        obeliskAngle *= -1;

        //distance in meters
        distanceToGoal = Math.hypot(x, y) / 39.37;
    }

    public ArrayList<MainSlot> determineOrder(@NonNull ArrayList<MainSlot.SlotState> pattern) {

        ArrayList<MainSlot> result1 = new ArrayList<>();
        Set<MainSlot> used = new HashSet<>();
        for (MainSlot.SlotState wanted : pattern) {
            for (MainSlot slot : slots) {
                if (slot.getColorState() == wanted && used.add(slot)) {
                    result1.add(slot);
                    break;
                }
            }
        }

        if (result1.size() >= 3) {
            return result1;
        }

        // Not full - add slots that are full first, then empty ones
        ArrayList<MainSlot> orderedResult = new ArrayList<>();

        // Add full slots first
        for (MainSlot slot : slots) {
            if (slot.getColorState() != MainSlot.SlotState.EMPTY) {
                orderedResult.add(slot);
            }
        }

        // Then add empty slots
        for (MainSlot slot : slots) {
            if (slot.getColorState() == MainSlot.SlotState.EMPTY) {
                orderedResult.add(slot);
            }
        }

        return orderedResult;
    }

    public Command Shoot() {

        Supplier<MainSlot> first = () -> result.get().get(0);
        Supplier<MainSlot> second = () -> result.get().get(1);
        Supplier<MainSlot> third = () -> result.get().get(2);

        return new LambdaCommand()
                .setStart(
                        new SequentialGroup(
                                first.get().basicShootDown().named("Result 1 " + first.get().getKickerServoName()),
                                second.get().basicShootDown().named("Result 2 " + second.get().getKickerServoName()),
                                third.get().basicShoot().named("Result 3 " + third.get().getKickerServoName())
                        ).setInterruptible(false).addRequirements(this));

    }


    public Command SetAllSlotState(MainSlot.ServoState state) {
        return new ParallelGroup(
                LeftSlot.INSTANCE.setServoState(state),
                RightSlot.INSTANCE.setServoState(state),
                BackSlot.INSTANCE.setServoState(state)
        );
    }
    

    public enum AllianceColor {
        RED,
        BLUE
    }

}
