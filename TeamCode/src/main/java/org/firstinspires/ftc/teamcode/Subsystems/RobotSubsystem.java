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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class RobotSubsystem extends SubsystemGroup {

    public static final RobotSubsystem INSTANCE = new RobotSubsystem();


    private RobotSubsystem() {
        super(
                OuttakeSubsystem.INSTANCE,
                VisionSubsystem.INSTANCE,
                IntakeSubsystem.INSTANCE,
                BackSlot.INSTANCE,
                LeftSlot.INSTANCE,
                RightSlot.INSTANCE
        );
    }

    private static ArrayList<MainSlot.SlotState> lastShot = new ArrayList<>();
    private static ArrayList<MainSlot.SlotState> pattern = new ArrayList<>(Arrays.asList(null, null, null));
    private static boolean patternFull = false;

    private static Pose previousPose = new Pose();
    ElapsedTime loopTimer = new ElapsedTime();
    private static AllianceColor allianceColor = AllianceColor.BLUE;


    private double distanceToGoalInches = 0;
    private double goalAngle = 0;
    private double obeliskAngle = 0;

    private static final ArrayList<MainSlot> slots = new ArrayList<>(Arrays.asList(BackSlot.INSTANCE, LeftSlot.INSTANCE, RightSlot.INSTANCE));

    private static int ballsShotLastSequence = 0;
    private static boolean enablePatternShifting = true;



    @Override
    public void initialize() {}


    @Override
    public void periodic() {

        loopTimer.reset();


        //Handles turret aiming
        if(allSlotsEmpty()){
            OuttakeSubsystem.INSTANCE.setTurretTargetAngle(0);
        } else {
            updateDistanceAndAngle();
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
        }



        //Handles pattern updating
        if (pattern.contains(null)) {
            pattern = VisionSubsystem.INSTANCE.getPattern();
            patternFull = !pattern.contains(null);
        }

        ActiveOpMode.telemetry().addData("Loop Times (ms) ", loopTimer.milliseconds());
        ActiveOpMode.telemetry().addData("Last Shot Num: ", ballsShotLastSequence);
        ActiveOpMode.telemetry().addData("Last Shot: ", lastShot);
        ActiveOpMode.telemetry().addData("Used Pattern: ", shift(pattern, ballsShotLastSequence > 0 && ballsShotLastSequence <= 2 ? 3 - ballsShotLastSequence : 0));
        ActiveOpMode.telemetry().update();

    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public void setAllianceColor(AllianceColor allianceColor) {
        RobotSubsystem.allianceColor = allianceColor;
    }

    public void resetPattern(){
        VisionSubsystem.INSTANCE.resetPattern();
        pattern = new ArrayList<>(Arrays.asList(null, null, null));
        patternFull = false;
    }

    public double getDistanceToGoalInches() {
        return distanceToGoalInches;
    }

    public boolean getPatternFull() {
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

        //distance in INCHES
        distanceToGoalInches = Math.hypot(x, y);
    }

    public ArrayList<MainSlot> determineOrder(@NonNull ArrayList<MainSlot.SlotState> pattern) {

        ArrayList<MainSlot.SlotState> usedPattern;

        // Shift pattern only if enabled and we had a partial sequence last time
        if(enablePatternShifting && ballsShotLastSequence > 0 && ballsShotLastSequence <= 2){
            usedPattern = shift(pattern, 3 - ballsShotLastSequence);
        } else {
            // If shifting disabled or we shot 0 or 3 balls, pattern repeats - no shift needed
            usedPattern = pattern;
        }

        // Reset counter for THIS sequence
        int currentSequenceMatches = 0;

        ArrayList<MainSlot> result1 = new ArrayList<>();
        Set<MainSlot> used = new HashSet<>();
        for (MainSlot.SlotState wanted : usedPattern) {
            for (MainSlot slot : slots) {
                if (slot.getColorState().equals(wanted) && used.add(slot)) {
                    result1.add(slot);
                    currentSequenceMatches++;  // Use local counter
                    break;
                }
            }
        }

        if (result1.size() == 3) {
            ballsShotLastSequence = currentSequenceMatches;  // Update for next time
            return result1;
        }

        // Not full - add slots that are full first, then empty ones
        ArrayList<MainSlot> orderedResult = new ArrayList<>();
        used = new HashSet<>();

        currentSequenceMatches = 0;
        // Add full slots first
        for (MainSlot slot : slots) {
            if (slot.getColorState() != MainSlot.SlotState.EMPTY && used.add(slot)) {
                orderedResult.add(slot);
                lastShot.add(slot.getColorState());
                currentSequenceMatches++;
            }
        }

        // Then add empty slots
        for (MainSlot slot : slots) {
            if (slot.getColorState() == MainSlot.SlotState.EMPTY && used.add(slot)) {
                orderedResult.add(slot);
            }
        }

        ballsShotLastSequence = currentSequenceMatches;  // Update for next time
        return orderedResult;
    }

    public Command Shoot() {
        // Capture the order at command creation time
        final ArrayList<MainSlot> shootOrder = new ArrayList<>();

        return new SequentialGroup(
                new LambdaCommand()
                        .setStart(() -> {
                            BackSlot.INSTANCE.readSlot();
                            RightSlot.INSTANCE.readSlot();
                            LeftSlot.INSTANCE.readSlot();

                            // Determine order after reading slots
                            shootOrder.clear();
                            shootOrder.addAll(determineOrder(pattern));
                        }),
                new LambdaCommand()
                        .setStart(() -> {
                            shootOrder.get(0).basicShootDown().named("Result 1 " + shootOrder.get(0).getKickerServoName()).run();
                        })
                        .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting")),
                new LambdaCommand()
                        .setStart(() -> {
                            shootOrder.get(1).basicShootDown().named("Result 2 " + shootOrder.get(1).getKickerServoName()).run();
                        })
                        .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting")),
                new LambdaCommand()
                        .setStart(() -> {
                            shootOrder.get(2).basicShoot().named("Result 3 " + shootOrder.get(2).getKickerServoName()).run();
                        })
                        .setIsDone(() -> !CommandManager.INSTANCE.hasCommandsUsing("Shooting")),
                new InstantCommand(() -> {
                    lastShot.clear();
                    lastShot.add(shootOrder.get(0).getColorState());
                    lastShot.add(shootOrder.get(1).getColorState());
                    lastShot.add(shootOrder.get(2).getColorState());
                })
        );

    }

    public ArrayList<MainSlot.SlotState> getPattern(){
        return pattern;
    }

    public Pose getPreviousPose() {
        return previousPose;
    }

    public void setPreviousPose(Pose previousPose) {
        RobotSubsystem.previousPose = previousPose;
    }

    public Command SetAllSlotState(MainSlot.ServoState state) {
        return new ParallelGroup(
                LeftSlot.INSTANCE.setServoState(state),
                RightSlot.INSTANCE.setServoState(state),
                BackSlot.INSTANCE.setServoState(state)
        );
    }

    public static ArrayList<MainSlot.SlotState> shift(ArrayList<MainSlot.SlotState> list, int shiftBy){

        shiftBy = list.size() - shiftBy;

        ArrayList<MainSlot.SlotState> shiftedList = new ArrayList<>(Arrays.asList(null, null, null)); //Not modularized

        for(int i = 0; i < list.size(); i++){
            shiftedList.set(i, list.get(((i + shiftBy) % list.size())));
        }

        return shiftedList;

    }

    public boolean allSlotsEmpty(){
        return LeftSlot.INSTANCE.getColorState() == MainSlot.SlotState.EMPTY &&
                RightSlot.INSTANCE.getColorState() == MainSlot.SlotState.EMPTY &&
                BackSlot.INSTANCE.getColorState() == MainSlot.SlotState.EMPTY;
    }

    public boolean isPatternShiftingEnabled() {
        return enablePatternShifting;
    }

    public void setPatternShiftingEnabled(boolean enabled) {
        enablePatternShifting = enabled;
    }

    public enum AllianceColor {
        RED,
        BLUE
    }

}