package org.firstinspires.ftc.teamcode.OpModes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Util.Subsystems.BetterVisionTM;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSortingSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Slot;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.util.ArrayList;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;


//Written by Noah Nottingham - 6566 Circuit Breakers

@TeleOp(name = "Functional Teleop", group = "Main") //The name and group
@Configurable
public class NextFTCTeleop extends NextFTCOpMode {



    public static UniConstants.teamColor color = UniConstants.teamColor.BLUE;

    static boolean isSlowed = false;

    static double distanceToGoalInMeters = 0.0;
    static double deltaAngle = 0.0;

    public static UniConstants.loggingState logState = UniConstants.loggingState.ENABLED;

    JoinedTelemetry joinedTelemetry;

    public static boolean usingVision = false;
    public static boolean botCentric = false;

    //All different subsystems
    private static BetterVisionTM vision = new BetterVisionTM();
    private static IntakeSortingSubsystem intake = new IntakeSortingSubsystem();
    private static TurretSubsystem turret = new TurretSubsystem();
    private static MecDriveSubsystem mecDrive = new MecDriveSubsystem();

    ArrayList<UniConstants.slotState> pattern = new ArrayList<>(List.of(null, null, null));

    CommandManager manager;

    Timer driverTimer = new Timer();
    Timer shooterTimer = new Timer();

    {
        addComponents(
                CommandManager.INSTANCE,
                new SubsystemComponent(turret, intake, mecDrive, vision)
        ); //Subsystems
    }

    @Override
    public void onInit() {
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        manager = CommandManager.INSTANCE;
    }

    @Override
    public void onWaitForStart() {
        if (gamepad1.a) {
            color = UniConstants.teamColor.RED;
        } else if (gamepad1.b) {
            color = UniConstants.teamColor.BLUE;
        }

        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("B for Blue, A for Red ");
        joinedTelemetry.addData("Current Team Color ", color);
        joinedTelemetry.update();
    }


    @Override
    public void onStartButtonPressed() {

        vision.setColor(color);

        turret.setTargetVelocity(2200);
        mecDrive.startTele();
    }

    @Override
    public void onUpdate() {

        isSlowed = gamepad1.left_bumper;

        //Spin active forward
        if (gamepad1.right_trigger > 0) {
            intake.forwardIntake();
            intake.enableActive();
        } //Reverse Active and Spin
        else if (gamepad1.left_trigger > 0) {
            intake.reverseIntake();
            intake.enableActive();
        }
        else {
            intake.disableActive();
        }

        if(gamepad1.a){
            turret.setTargetVelocity(3000);
        }

        if(gamepad1.b){
            turret.setTargetVelocity(0);
        }

        //Able to switch between driver and robot centric
        if(gamepad1.y && driverTimer.getTimeSeconds() > .5){
            botCentric = !botCentric;
            driverTimer.reset();
        }

        //Shooting command
        if(gamepad1.right_bumper && shooterTimer.getTimeSeconds() > 1.5) {
            manager.scheduleCommand(intake.shoot(pattern));
            shooterTimer.reset();
        }

        if(intake.allFull()){
            gamepad1.rumble(250);
        } else {
            gamepad1.stopRumble();
        }

        //If pattern hasn't been assigned yet
        if(pattern.contains(null)){
            pattern = vision.getPattern();
        }

        //TODO: Determine what is better
        if(usingVision){
            distanceToGoalInMeters = vision.getDistanceToGoal();
            deltaAngle = vision.getDeltaAngle();
        }
        else{
            distanceToGoalInMeters = mecDrive.updateDistanceAndAngle();
            deltaAngle = mecDrive.getCalculatedTurretAngle();
        }

        //TODO: Still have to integrate look up table or linreg for power as a function of distance
        //TODO: Integrate actual turret control, right now I believe it will not work (also its not even being set)

        mecDrive.updateTeleop(
                -gamepad1.left_stick_y * (isSlowed ? .25 : 1), //Forward/Backward
                -gamepad1.left_stick_x * (isSlowed ? .25 : 1), //Left/Right Rotation
                -gamepad1.right_stick_x * (isSlowed ? .25 : 1), //Left/Right Strafe
                botCentric
        );


        manager.run();

        joinedTelemetry.addData("Bot Centric ", botCentric);
        joinedTelemetry.addData("Pattern ", pattern);
        joinedTelemetry.addData("Current Commands ", manager.snapshot());
//        for(Slot slot : intake.slots){
//            slot.sendTelemetry(UniConstants.loggingState.ENABLED);
//        }
        turret.sendTelemetry(UniConstants.loggingState.ENABLED);
        joinedTelemetry.update();

    }




    }
