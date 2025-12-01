package org.firstinspires.ftc.teamcode.OpModes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.Subsystems.BetterVisionTM;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDriveSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Subsystems.RotaryIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.ftc.NextFTCOpMode;


//Written by Noah Nottingham - 6566 Circuit Breakers

@TeleOp(name = "Functional Teleop", group = "Main") //The name and group
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

    Timer rotaryTimer = new Timer();


    //All different subsystems
    private static BetterVisionTM vision;
    private static RotaryIntakeSubsystem rotaryIntake;
    private static OuttakeSubsystem outtake;
    private static MecDriveSubsystem mecDrive;

    @Override
    public void onInit() {
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
//        vision = new BetterVisionTM(hardwareMap, joinedTelemetry, logState);
        rotaryIntake = new RotaryIntakeSubsystem(hardwareMap, joinedTelemetry, color);
        outtake = new OuttakeSubsystem(hardwareMap, joinedTelemetry, color);
        mecDrive = new MecDriveSubsystem(hardwareMap, joinedTelemetry, color);
        //mecDrive.resetPinpoint();
        outtake.resetMotors();
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

        rotaryIntake.setColor(color);
        outtake.setColor(color);
        outtake.setTurretTargetAngle(0);
        mecDrive.setColor(color);
        mecDrive.startTele();
//        mecDrive.setPose(new Pose(72, 72, Math.toRadians(90)));


    }

    @Override
    public void onUpdate() {

        isSlowed = gamepad1.left_bumper;

        //Spin active forward, if the front slot is full, move to the next one.
        if (gamepad1.right_trigger > 0) {
            rotaryIntake.forwardIntake();
            rotaryIntake.enableActive();
        } //Reverse Active and Spin
        else if (gamepad1.left_trigger > 0) {
            rotaryIntake.reverseIntake();
            rotaryIntake.enableActive();
        }
        else {
            rotaryIntake.disableActive();
        }

        if(gamepad1.a){
            outtake.setLauncherTargetVelo(2000);
        }

        if(gamepad1.b){
            outtake.setLauncherTargetVelo(0);
            outtake.setPower(0);
        }

        if (gamepad1.right_bumper && rotaryTimer.getTimeSeconds() > 1) {
            //Transfer command here
//            rotaryIntake.toggleServo();
//            rotaryTimer.reset();
        }

        if(rotaryIntake.stateFront == RotaryIntakeSubsystem.servoState.OUTTAKE){
            gamepad1.rumble(250);
        } else {
            gamepad1.stopRumble();
        }






        //Velocity should be constantly interpolated via apriltag localization data right? Heavy comp power
        //Velocity should only be queried when ready to shoot.
        //TODO: Generate linear regression to determine velocity for given positions
        //Exit velocity will be a function of the power put into the motor (PDFL)

        distanceToGoalInMeters = mecDrive.updateDistanceAndAngle();
//            outtake.setLauncherTargetVelo(outtake.getTargetVelocity(distanceToGoalInMeters));
//            outtake.setTurretTargetAngle(mecDrive.getCalculatedTurretAngle() - 90);


        mecDrive.updateTeleop(
                -gamepad1.left_stick_y * (isSlowed ? .25 : 1), //Forward/Backward
                -gamepad1.left_stick_x * (isSlowed ? .25 : 1), //Left/Right Rotation
                -gamepad1.right_stick_x * (isSlowed ? .25 : 1), //Left/Right Strafe
                true
        );

        rotaryIntake.periodic(); //Updater for rotary + intake
        outtake.periodic(); //updater for launcher and turret
//        vision.periodic(); //updater for vision
        mecDrive.periodic(); //updater for Mecanum drive and follower

//        rotaryIntake.sendTelemetry(logState);
        outtake.sendTelemetry(logState);
        mecDrive.sendTelemetry(logState);
        telemetry.addData("Distance to Goal ", distanceToGoalInMeters);
        telemetry.update();

        }


    }
