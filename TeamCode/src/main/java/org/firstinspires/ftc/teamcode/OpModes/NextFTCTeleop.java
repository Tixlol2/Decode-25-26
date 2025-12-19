package org.firstinspires.ftc.teamcode.OpModes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Subsystems.BetterVisionTM;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeSortingSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;


//Written by Noah Nottingham - 6566 Circuit Breakers

@TeleOp(name = "Functional Teleop", group = "Main") //The name and group
@Configurable
public class NextFTCTeleop extends NextFTCOpMode {



    UniConstants.teamColor color = UniConstants.teamColor.BLUE;

    static boolean isSlowed = false;

    static double distanceToGoalInMeters = 0.0;

    public static UniConstants.loggingState logState = UniConstants.loggingState.ENABLED;

    JoinedTelemetry joinedTelemetry;

    Timer rotaryTimer = new Timer();




    //All different subsystems
    private static BetterVisionTM vision;
    private static IntakeSortingSubsystem intake;
    private static TurretSubsystem outtake;
    private static MecDriveSubsystem mecDrive;



    @Override
    public void onInit() {
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        intake = new IntakeSortingSubsystem(hardwareMap, joinedTelemetry);

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
            intake.setServoState(intake.frontServo, UniConstants.FLICKER_UP).schedule();
        }

        if(gamepad1.b){
            intake.setServoState(intake.frontServo, UniConstants.FLICKER_DOWN).schedule();
        }

        //Shooting command
        if (gamepad1.right_bumper ) {


        }


        }


    {
        addComponents(
                CommandManager.INSTANCE,
                new SubsystemComponent(intake, outtake, mecDrive)
        ); //Subsystems
    }

    }
