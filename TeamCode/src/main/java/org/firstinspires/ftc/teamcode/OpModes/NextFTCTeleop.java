package org.firstinspires.ftc.teamcode.OpModes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Util.Subsystems.BetterVisionTM;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.RotaryIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.ftc.NextFTCOpMode;


//Written by Noah Nottingham - 6566 Circuit Breakers

@TeleOp(name = "Functional Teleop", group = "Driver") //The name and group
@Configurable
public class NextFTCTeleop extends NextFTCOpMode {

    {
        addComponents(); //Subsystems
    }

    UniConstants.teamColor color = UniConstants.teamColor.BLUE;

    static boolean isSlowed = false;

    static double distanceToGoalInMeters = 0.0;

    public static UniConstants.loggingState logState = UniConstants.loggingState.ENABLED;

    JoinedTelemetry joinedTelemetry;


    //All different subsystems
    private static BetterVisionTM vision;
    private static RotaryIntakeSubsystem rotaryIntake;
    private static OuttakeSubsystem outtake;
    private static MecDriveSubsystem mecDrive;

    @Override
    public void onInit() {
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        vision = new BetterVisionTM(hardwareMap, joinedTelemetry, logState);
    }

    @Override public void onWaitForStart() {
        if(gamepad1.a){
            color = UniConstants.teamColor.RED;
        }else if (gamepad1.b){
            color = UniConstants.teamColor.BLUE;
        } else if (gamepad1.y){
            rotaryIntake = new RotaryIntakeSubsystem(hardwareMap, joinedTelemetry, color);
            outtake = new OuttakeSubsystem(hardwareMap, joinedTelemetry, color);
            mecDrive = new MecDriveSubsystem(hardwareMap, joinedTelemetry, color);
            mecDrive.resetPinpoint();
        }



        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("B for Blue, A for Red ");
        joinedTelemetry.addData("Current Team Color ", color);
        joinedTelemetry.addLine("IF correct color selected; press Y");
        joinedTelemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        for (ColorSensor sensor : rotaryIntake.colorSensors) {
            sensor.enableLed(true);
        }

        mecDrive.startTele();


    }

    @Override
    public void onUpdate() {

        isSlowed = gamepad1.right_bumper;

        if (gamepad1.a && !rotaryIntake.allFull()) {
            rotaryIntake.enableActive();
            if (rotaryIntake.isFull(rotaryIntake.slots.get(0))) {
                if (!(rotaryIntake.getError() > 50)) {
                    rotaryIntake.setRotaryTargetPosition(rotaryIntake.getRotaryCurrentPosition() + UniConstants.SPACE_BETWEEN_ROTARY_SLOTS);
                }
            }
        } else {
            rotaryIntake.disableActive();
        }

        //Velocity should be constantly interpolated via apriltag localization data right? Heavy comp power
        //Velocity should only be queried when ready to shoot.
        //TODO: Generate linear regression to determine velocity for given positions
        //Exit velocity will be a function of the power put into the motor (PDFL)


        outtake.setLauncherTargetVelo(outtake.getTargetVelocity(mecDrive.updateDistanceAndAngle(color)));
        outtake.setTurretTargetAngle(mecDrive.getCalculatedTurretAngle());
        distanceToGoalInMeters = mecDrive.updateDistanceAndAngle(color);


        mecDrive.updateTeleop(
                gamepad1.left_stick_y * (isSlowed ? .5 : 1), //Forward/Backward
                gamepad1.left_stick_x * (isSlowed ? .5 : 1), //Left/Right Rotation
                gamepad1.right_stick_x * (isSlowed ? .5 : 1), //Left/Right Strafe
                true
        );

        rotaryIntake.periodic(); //Updater for rotary + intake
        outtake.periodic(); //updater for launcher and turret
        vision.periodic(); //updater for vision
        mecDrive.periodic(); //updater for Mecanum drive and follower

        rotaryIntake.sendTelemetry(logState);
        outtake.sendTelemetry(logState);
        mecDrive.sendTelemetry(logState);
        telemetry.update();

    }




}
