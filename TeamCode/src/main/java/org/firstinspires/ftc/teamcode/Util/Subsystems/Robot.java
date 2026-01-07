package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.util.ArrayList;
import java.util.Arrays;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.ftc.ActiveOpMode;

public class Robot extends SubsystemGroup {

    public static final Robot INSTANCE = new Robot();

    public static ArrayList<UniConstants.slotState> pattern = new ArrayList<>(Arrays.asList(null, null, null));
    public static boolean patternFull = false;

    public static UniConstants.teamColor color = UniConstants.teamColor.BLUE;

    public static UniConstants.loggingState loggingState = UniConstants.loggingState.ENABLED;

    public static boolean turretForward = true;
    public static boolean aimAtGoal = false;

    private double distanceToGoal = 0;
    private JoinedTelemetry joinedTelemetry;

    public static boolean inTeleop = false;

    ElapsedTime loopTimer = new ElapsedTime();


    private Robot() {
        super(
                MecDriveSubsystem.INSTANCE,
                TurretSubsystem.INSTANCE,
                IntakeSortingSubsystem.INSTANCE,
                BetterVisionTM.INSTANCE
        );
    }

    @Override
    public void initialize(){

        joinedTelemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());


        turretForward = true;
        aimAtGoal = false;

    }

    @Override
    public void periodic(){

        loopTimer.reset();

        distanceToGoal = MecDriveSubsystem.INSTANCE.getDistanceToGoal();
        //Handles turret aiming
        TurretSubsystem.INSTANCE.setTargetAngle(turretForward ? 0 : (aimAtGoal ? -MecDriveSubsystem.INSTANCE.getCalculatedTurretAngle() : -MecDriveSubsystem.INSTANCE.getObeliskAngle()));



        //Handles pattern updating
        if(pattern.contains(null)){
            pattern = BetterVisionTM.INSTANCE.getPattern();
            patternFull = !pattern.contains(null);
        }


        joinedTelemetry.addData("Loop Time (ms) ", loopTimer.milliseconds());
        joinedTelemetry.addData("Pattern: ", pattern);
        joinedTelemetry.addData("Pattern Full: ", patternFull);

        joinedTelemetry.update();

    }

    public double getDistanceToGoal(){
        return distanceToGoal;
    }

    public JoinedTelemetry getJoinedTelemetry(){
        return joinedTelemetry;
    }

    public Command ShootCreep(double distanceCreepInches, double maxPower){
        ArrayList<IntakeSortingSubsystem.Slot> order = IntakeSortingSubsystem.INSTANCE.determineOrder(pattern);
        return new SequentialGroup(
                FaceGoal(),
                IntakeSortingSubsystem.INSTANCE.Shoot(order.get(0), false),
                MecDriveSubsystem.INSTANCE.PushForward(maxPower, distanceCreepInches, false),
                IntakeSortingSubsystem.INSTANCE.Shoot(order.get(1), false),
                MecDriveSubsystem.INSTANCE.PushForward(maxPower, distanceCreepInches, false),
                IntakeSortingSubsystem.INSTANCE.Shoot(order.get(2), false)
        );
    }

    public Command ShootWait(double waitTime){
        ArrayList<IntakeSortingSubsystem.Slot> order = IntakeSortingSubsystem.INSTANCE.determineOrder(pattern);
        return new SequentialGroup(
                IntakeSortingSubsystem.INSTANCE.Shoot(order.get(0), false),
                new Delay(waitTime),
                IntakeSortingSubsystem.INSTANCE.Shoot(order.get(1), false),
                new Delay(waitTime),
                IntakeSortingSubsystem.INSTANCE.Shoot(order.get(2), true)
                );
    }

    public Command TurretForward(){
        return new InstantCommand(() -> {turretForward = true; aimAtGoal = true;});
    }

    public Command TurretGoal(){
        return new InstantCommand(() -> {turretForward = false; aimAtGoal = true;});
    }

    public Command TurretObelisk(){
        return new InstantCommand(() -> {turretForward = false; aimAtGoal = false;});
    }

    public Command FaceGoal(){
        return new ParallelGroup(
                TurretGoal(),
                MecDriveSubsystem.INSTANCE.TurnTo(color == UniConstants.teamColor.BLUE ? UniConstants.ANGLE_BLUE_GOAL_DEGREES : UniConstants.ANGLE_RED_GOAL_DEGREES)
        );
    }

    public Command ShootEndPath(PathChain path){
        return new SequentialGroup(
                MecDriveSubsystem.INSTANCE.FollowPath(path, true),
                ShootCreep(5, .5)
        );
    }

    public Command FollowPathActive(PathChain path, boolean holdEnd){
        return new ParallelGroup(
                IntakeSortingSubsystem.INSTANCE.runActive(),
                MecDriveSubsystem.INSTANCE.FollowPath(path, holdEnd)
        );
    }

    public Command FollowPathActive(PathChain path, boolean holdEnd, double maxPower){
        return new ParallelGroup(
                IntakeSortingSubsystem.INSTANCE.runActive(),
                MecDriveSubsystem.INSTANCE.FollowPath(path, holdEnd, maxPower)
        );
    }

    public Command Park(){
        return new ParallelGroup(
                StopSubsystems(),
                MecDriveSubsystem.INSTANCE.FollowPath(MecDriveSubsystem.INSTANCE.createParkPath(), true)
        );
    }

    public Command StopSubsystems(){
        return new ParallelGroup(
                TurretForward(),
                IntakeSortingSubsystem.INSTANCE.stopActive(),
                TurretSubsystem.INSTANCE.SetMotorPower(0)
        );
    }



}


