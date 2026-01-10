package org.firstinspires.ftc.teamcode.Util.Subsystems;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.util.ArrayList;
import java.util.Arrays;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class Robot extends SubsystemGroup {

    public static final Robot INSTANCE = new Robot();

    public static ArrayList<UniConstants.slotState> pattern = new ArrayList<>(Arrays.asList(null, null, null));
    public static boolean patternFull = false;

    public static UniConstants.teamColor color = UniConstants.teamColor.BLUE;

    public static UniConstants.loggingState loggingState = UniConstants.loggingState.ENABLED;

    private double distanceToGoal = 0;
    private JoinedTelemetry joinedTelemetry;

    public static boolean inTeleop = true;
    public static boolean automatedDrive = false;

    public static Timer shotTimer = new Timer();

    ElapsedTime loopTimer = new ElapsedTime();


    public static double standardWaitTime = .75;

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
        setGlobals();

    }

    @Override
    public void periodic(){

        loopTimer.reset();

        distanceToGoal = MecDriveSubsystem.INSTANCE.getDistanceToGoal();
        //Handles turret aiming
        switch(TurretSubsystem.state){
            case FORWARD:
                TurretSubsystem.INSTANCE.setTargetAngle(0);
                break;
            case GOAL:
                TurretSubsystem.INSTANCE.setTargetAngle(MecDriveSubsystem.INSTANCE.getGoalAngle());
                break;
            case OBELISK:
                TurretSubsystem.INSTANCE.setTargetAngle(MecDriveSubsystem.INSTANCE.getObeliskAngle());
                break;
        }


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
                IntakeSortingSubsystem.INSTANCE.Shoot(order.get(2), true),
                new InstantCommand(() -> Robot.shotTimer.reset())
        );
    }

    public Command Shoot(){
        ArrayList<IntakeSortingSubsystem.Slot> order = IntakeSortingSubsystem.INSTANCE.determineOrder(pattern);
        return new SequentialGroup(
                IntakeSortingSubsystem.INSTANCE.Shoot(order.get(0), false),
                IntakeSortingSubsystem.INSTANCE.Shoot(order.get(1), false),
                IntakeSortingSubsystem.INSTANCE.Shoot(order.get(2), true),
                new InstantCommand(() -> Robot.shotTimer.reset())
        );
    }

    public Command TurretForward(){
        return new LambdaCommand()
                .setStart(() -> {TurretSubsystem.INSTANCE.setTurretState(TurretSubsystem.turretState.FORWARD);})
                .setIsDone(TurretSubsystem.INSTANCE::turretFinished);
    }

    public Command TurretGoal(){
        return new LambdaCommand()
                .setStart(() -> {TurretSubsystem.INSTANCE.setTurretState(TurretSubsystem.turretState.GOAL);})
                .setIsDone(TurretSubsystem.INSTANCE::turretFinished);
    }

    public Command TurretObelisk(){
        return new LambdaCommand()
                .setStart(() -> {TurretSubsystem.INSTANCE.setTurretState(TurretSubsystem.turretState.OBELISK);})
                .setIsDone(TurretSubsystem.INSTANCE::turretFinished);
    }

    public Command FaceGoal(){
        return new ParallelGroup(
                TurretGoal(),
                new TurnTo(color == UniConstants.teamColor.BLUE ? Angle.fromDeg(UniConstants.ANGLE_BLUE_GOAL_DEGREES) : Angle.fromDeg(UniConstants.ANGLE_RED_GOAL_DEGREES))
        );
    }

    public Command Park(){
        return new ParallelGroup(
                StopSubsystems(),
                new FollowPath(MecDriveSubsystem.INSTANCE.createParkPath(), true)
        );
    }

    public Command StopSubsystems(){
        return new ParallelGroup(
                TurretForward(),
                IntakeSortingSubsystem.INSTANCE.stopActive(),
                TurretSubsystem.INSTANCE.SetMotorPower(0)
        );
    }

    public Command ActivePath(PathChain path){
        return new ParallelGroup(
                IntakeSortingSubsystem.INSTANCE.runActive(),
                new FollowPath(path)
        );
    }

    public Command ActivePath(PathChain path, boolean holdEnd, double maxPower){
        return new ParallelGroup(
                IntakeSortingSubsystem.INSTANCE.runActive(),
                new FollowPath(path, holdEnd, maxPower)
        );
    }

    public void setGlobalTelemetry(){
        MecDriveSubsystem.INSTANCE.setTelemetry(joinedTelemetry);
        TurretSubsystem.INSTANCE.setTelemetry(joinedTelemetry);
        IntakeSortingSubsystem.INSTANCE.setTelemetry(joinedTelemetry);
        BetterVisionTM.INSTANCE.setTelemetry(joinedTelemetry);
    }

    public void setGlobalColor(){
        MecDriveSubsystem.INSTANCE.setColor(color);
        BetterVisionTM.INSTANCE.setColor(color);

    }

    public void setGlobals(){
        setGlobalTelemetry();
        setGlobalColor();
    }

    public Command StartTeleop(){
        return new SequentialGroup(
                new InstantCommand(() -> inTeleop = true),
                TurretSubsystem.INSTANCE.SetMotorPower(0),
                new InstantCommand(MecDriveSubsystem.INSTANCE::startTele),
                IntakeSortingSubsystem.INSTANCE.stopActive(),
                new InstantCommand(() -> MecDriveSubsystem.INSTANCE.setStartPose(Poses.blueGoalTopStartFacing)),
                TurretGoal()
        );
    }

    public boolean withinRange(double value, double target, double range){
        return Math.abs(value - target) < range;
    }


    public Command PathShoot(){
        return new SequentialGroup(
                Robot.INSTANCE.TurretForward(),
                new ParallelGroup(
                        //new IfElseCommand(() -> TurretSubsystem.debugPower == .75, new NullCommand(), new ParallelGroup(new Delay(5), TurretSubsystem.INSTANCE.SetMotorPower(.75))),
                        new FollowPath(MecDriveSubsystem.INSTANCE.createShootingPath(), true)
                ),
                Robot.INSTANCE.ShootWait(.75),
                new InstantCommand(() -> {
                    follower().breakFollowing();
                    if (Robot.inTeleop) {
                        follower().startTeleopDrive();
                    }
                })
        );
    }




}


