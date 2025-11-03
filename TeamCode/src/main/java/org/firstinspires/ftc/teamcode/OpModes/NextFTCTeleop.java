package org.firstinspires.ftc.teamcode.OpModes;


import com.bylazar.configurables.annotations.Configurable;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.BetterVisionTM;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.ArrayList;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;


//Written by Noah Nottingham - 6566 Circuit Breakers

@TeleOp(name = "Functional Teleop", group = "Driver") //The name and group
@Configurable
public class NextFTCTeleop extends NextFTCOpMode {

    {
        addComponents(); //Subsystems
    }



    UniConstants.teamColor color = UniConstants.teamColor.BLUE;

    public static int obeliskApriltagID = 21;

    boolean isSlowed = false;

    double distanceToGoalInMeters = 0.0;

    Follower follower;

    DcMotorEx active;
    DcMotorEx launcher;
    DcMotorEx rotary;
    DcMotorEx turret;

    int rotaryCurrentPosition = 0;
    int rotaryTargetPosition = 0;

    public static double pR = 0, dR = 0, lR = 0, fR = 0;
    PDFLController rotaryController = new PDFLController(pR, dR, fR, lR);

    public static double pL = 0, dL = 0, lL = 0, fL = 0;
    PDFLController launcherController = new PDFLController(pL, dL, fL, lL);
    public static double launcherTargetVelo = 0;

    public static double pT = 0, dT = 0, lT = 0, fT = 0;
    PDFLController turretController = new PDFLController(pT, dT, fT, lT);
    public static double turretTargetPosition = 0;

    public static double turretTargetAngle = 0;
    public static double rotaryPower = 0;
    public static double turretPower = 0;

    public static UniConstants.loggingState logState = UniConstants.loggingState.ENABLED;

    ArrayList<UniConstants.slotState> slots = new ArrayList<>(List.of(UniConstants.slotState.EMPTY, UniConstants.slotState.EMPTY, UniConstants.slotState.EMPTY));
    ArrayList<UniConstants.slotState> pattern = new ArrayList<>();
    ArrayList<ColorSensor> colorSensors = new ArrayList<>();

    Servo ballServo;
    public static double servoPos = 0;

    DcMotorSimple.Direction ACTIVE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    BetterVisionTM vision;

    @Override
    public void onInit() {

        JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        vision = new BetterVisionTM(hardwareMap, joinedTelemetry, logState);
        vision.initialize();


        follower = Constants.createFollower(hardwareMap);

        active = hardwareMap.get(DcMotorEx.class, UniConstants.ACTIVE_INTAKE_STRING);
        active.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        active.setDirection(ACTIVE_DIRECTION);

        launcher = hardwareMap.get(DcMotorEx.class, UniConstants.LAUNCHER_STRING);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret = hardwareMap.get(DcMotorEx.class, UniConstants.TURRET_ROTATION_STRING);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotary = hardwareMap.get(DcMotorEx.class, UniConstants.ROTARY_STRING);
        rotary.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotary.setDirection(UniConstants.ROTARY_DIRECTION);

        ballServo = hardwareMap.get(Servo.class, UniConstants.BALL_SERVO_STRING);

        colorSensors.addAll(
                List.of(
                        (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_FRONT_STRING)),
                        (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_RIGHT_STRING)),
                        (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_LEFT_STRING))
                )
        );

        if(gamepad1.a){
            color = UniConstants.teamColor.RED;
        }else if (gamepad1.b){
            color = UniConstants.teamColor.BLUE;
        }

        joinedTelemetry.addLine("CHANGE THIS IF NEED BE!!!! ");
        joinedTelemetry.addLine("B for Blue, A for Red ");
        joinedTelemetry.addData("Team Color ", color);
        joinedTelemetry.update();

        follower.setStartingPose(new Pose());
        follower.update();


        obeliskTargetPattern(obeliskApriltagID);
    }

    @Override
    public void onStartButtonPressed() {
        for (ColorSensor sensor : colorSensors) {
            sensor.enableLed(true);
        }

        follower.startTeleopDrive();
        follower.update();


    }

    @Override
    public void onUpdate() {

        isSlowed = gamepad1.right_bumper;

        if (gamepad1.a && !allFull()) {
            active.setPower((isSlowed ? .5 : 1));
            if (isFull(slots.get(0))) {
                if (!((Math.abs(rotaryTargetPosition - rotaryCurrentPosition)) > 50)) {
                    rotaryTargetPosition += UniConstants.SPACE_BETWEEN_ROTARY_SLOTS;
                }           }
        } else {
            active.setPower(0);
        }




        readSlots();



        //If blue, aim for blue goal else aim for red goal - this target position will be fed to servo
        //Is it possible to send data from auto to tele? like whether blue or not
        //Think about only doing this every 500ms or so so we arent constantly computing
        //Tradeoff of accuracy vs chub efficiency

        //Velocity should be constantly interpolated via apriltag localization data right? Heavy comp power
        //Velocity should only be queried when ready to shoot.
        //TODO: Generate linear regression to determine velocity for given positions
        //Exit velocity will be a function of the power put into the motor (PDFL)



        //Slot numbers are already known, 0 is ready to transfer into launcher, 1 is to the right, 2 is the last one.
        //So for obelisk PPG, 0 = P, 1 = P, 2 = G
        //If that is good, bool readyToLaunch = true
        //I need the robot to be built or cad to be done to actually do this without theory coding but shrug
        rotaryController.setTarget(rotaryTargetPosition);
        rotaryCurrentPosition = rotary.getCurrentPosition();
        rotaryController.update(rotaryCurrentPosition);
        rotary.setPower(rotaryPower);

        turretController.setTarget(turretTargetPosition);
        turretController.update(turret.getCurrentPosition());
        turret.setPower(turretPower);

        launcherController.setTarget(launcherTargetVelo);
        launcherController.update(launcher.getVelocity());
        launcher.setPower(launcherController.runPDFL(.05));

        distanceToGoalInMeters = getDistanceToGoalInMeters(color);
        turretTargetAngle = 0; //Vision.getTargetAngle

        ballServo.setPosition(servoPos);


        follower.setTeleOpDrive(
                gamepad1.left_stick_y * (isSlowed ? .5 : 1), //Forward/Backward
                gamepad1.left_stick_x * (isSlowed ? .5 : 1), //Left/Right Rotation
                gamepad1.right_stick_x * (isSlowed ? .5 : 1), //Left/Right Strafe
                true);
        follower.update();


        vision.periodic();
        sendTelemetry(logState);
        telemetry.update();

    }

    public void readSlots() {
        for (int i = 0; i < 3; i++) {
            double red = colorSensors.get(i).red();
            double green = colorSensors.get(i).green();
            double blue = colorSensors.get(i).blue();
            double alpha = colorSensors.get(i).alpha();
            if (((green > red) && (blue > red)) && (alpha < 5000)) {
                slots.set(i, UniConstants.slotState.GREEN);
            } else if (((red > green) && (blue > green)) && (alpha < 5000)) {
                slots.set(i, UniConstants.slotState.PURPLE);
            } else if (alpha > 5000){
                slots.set(i, UniConstants.slotState.BETWEEN);
            } else {
                slots.set(i, UniConstants.slotState.EMPTY);
            }
        }

    }

    public double getTargetVelocity(double distanceToGoalInMeters) {
        //https://www.desmos.com/calculator/yw7iis7m3w
        //https://medium.com/@vikramaditya.nishant/programming-a-decode-shooter-4ab114dac01f
        return Math.sqrt(
                ((9.81) * (Math.pow(distanceToGoalInMeters, 2)))
                        /
                        (Math.pow(2 * (Math.cos(Math.toRadians(UniConstants.ANGLE_OF_LAUNCHER_IN_DEGREES))), 2) * ((distanceToGoalInMeters * Math.tan(Math.toRadians(UniConstants.ANGLE_OF_LAUNCHER_IN_DEGREES))) - UniConstants.HEIGHT_TO_GOAL_WITH_CLEARANCE_METERS))
        );
    }

    public boolean isFull(UniConstants.slotState slot) {
        return (slot == UniConstants.slotState.PURPLE) || (slot == UniConstants.slotState.GREEN);
    }

    public boolean allFull() {

        for (UniConstants.slotState slot : slots) {
            if (!isFull(slot)) {
                return false;
            }
        }
        return true;
    }

    public void obeliskTargetPattern(int ID){
        switch(ID){
            case 21:
                pattern = new ArrayList<>(List.of(UniConstants.slotState.GREEN, UniConstants.slotState.PURPLE, UniConstants.slotState.PURPLE));
                break;
            case 22:
                pattern = new ArrayList<>(List.of(UniConstants.slotState.PURPLE, UniConstants.slotState.GREEN, UniConstants.slotState.PURPLE));
                break;
            case 23:
                pattern = new ArrayList<>(List.of(UniConstants.slotState.PURPLE, UniConstants.slotState.PURPLE, UniConstants.slotState.GREEN));
                break;
        }

    }


    public double getDistanceToGoalInMeters(UniConstants.teamColor color) {
        double x = 0,y = 1;
        switch (color){
            case BLUE:
                x = follower.getPose().getX() - Poses.blueGoal.getX();
                y = follower.getPose().getY() - Poses.blueGoal.getY();
                break;
            case RED:
                x = follower.getPose().getX() - Poses.redGoal.getX();
                y = follower.getPose().getY() - Poses.redGoal.getY();
        }

        turretTargetAngle = Math.toDegrees(Math.atan(x / y));
        return (Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))) / 39.37;



    }

    public double getTurretTargetPosition(double turretTargetAngle){

        //Ratio given in terms of motor/turret

        return (turretTargetAngle / UniConstants.MOTOR_TO_TURRET_RATIO) * UniConstants.TURRET_TICKS_PER_DEGREE;

    }

    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){

            case DISABLED:
                break;
            case ENABLED:
                telemetry.addData("Rotary Current Pos ", rotaryCurrentPosition);
                telemetry.addData("Rotary Target Pos ", rotaryTargetPosition);
                telemetry.addLine();
                telemetry.addData("Slot Front State ", slots.get(0));
                telemetry.addData("Slot Right State ", slots.get(1));
                telemetry.addData("Slot Left State ", slots.get(2));
                telemetry.addLine();
                telemetry.addData("Turret Target Pos ", getTurretTargetPosition(turretTargetAngle));
                telemetry.addData("Turret Target Angle ", turretTargetAngle);
                telemetry.addLine();
                telemetry.addData("Launcher Target Velo ", launcherTargetVelo);
                telemetry.addData("Calculated Launch Velocity ", getTargetVelocity(distanceToGoalInMeters));
                telemetry.addData("Distance To Goal In Meters ", distanceToGoalInMeters);
                telemetry.addLine();
                telemetry.addData("Pose X ", follower.getPose().getX());
                telemetry.addData("Pose Y ", follower.getPose().getY());
                telemetry.addData("Pose Heading ", follower.getPose().getHeading());
                break;
            case EXTREME:
                telemetry.addData("Rotary Current Pos ", rotaryCurrentPosition);
                telemetry.addData("Rotary Target Pos ", rotaryTargetPosition);
                telemetry.addLine();
                telemetry.addData("Slot Front State ", slots.get(0));
                telemetry.addData("Slot Front Green ", colorSensors.get(0).green());
                telemetry.addData("Slot Front Red ", colorSensors.get(0).red());
                telemetry.addData("Slot Front Blue ", colorSensors.get(0).blue());
                telemetry.addData("Slot Front Alpha ", colorSensors.get(0).alpha());
                telemetry.addLine();
                telemetry.addData("Slot Right State ", slots.get(1));
                telemetry.addData("Slot Right Green ", colorSensors.get(1).green());
                telemetry.addData("Slot Right Red ", colorSensors.get(1).red());
                telemetry.addData("Slot Right Blue ", colorSensors.get(1).blue());
                telemetry.addData("Slot Right Alpha ", colorSensors.get(1).alpha());
                telemetry.addLine();
                telemetry.addData("Slot Left State ", slots.get(2));
                telemetry.addData("Slot Left Green ", colorSensors.get(2).green());
                telemetry.addData("Slot Left Red ", colorSensors.get(2).red());
                telemetry.addData("Slot Left Blue ", colorSensors.get(2).blue());
                telemetry.addData("Slot Left Alpha ", colorSensors.get(2).alpha());
                telemetry.addLine();
                telemetry.addData("Obelisk Perfect Pattern ", pattern);
                telemetry.addLine();
                telemetry.addData("Launcher Target Velo ", launcherTargetVelo);
                telemetry.addData("Calculated Launch Velocity ", getTargetVelocity(distanceToGoalInMeters));
                telemetry.addData("Distance To Goal In Meters ", distanceToGoalInMeters);
                telemetry.addLine();
                telemetry.addData("Turret Target Pos ", getTurretTargetPosition(turretTargetAngle));
                telemetry.addData("Turret Target Angle ", turretTargetAngle);
                telemetry.addLine();
                telemetry.addData("Pose X ", follower.getPose().getX());
                telemetry.addData("Pose Y ", follower.getPose().getY());
                telemetry.addData("Pose Heading ", follower.getPose().getHeading());

                break;
        }
    }

}
