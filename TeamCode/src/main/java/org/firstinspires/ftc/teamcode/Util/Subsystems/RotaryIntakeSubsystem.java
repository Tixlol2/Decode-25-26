package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.util.ArrayList;

import dev.nextftc.core.subsystems.Subsystem;

@Configurable
public class RotaryIntakeSubsystem implements Subsystem {
    //Class variables
    JoinedTelemetry telemetry;
    UniConstants.teamColor color;
    public static boolean debug = false;
    public static boolean isEnabled = false;
    public static boolean isReversed = false;

    //Arraylists
    public ArrayList<ColorSensor> colorSensors = new ArrayList<>();
    public ArrayList<UniConstants.slotState> slots = new ArrayList<>(List.of(UniConstants.slotState.EMPTY, UniConstants.slotState.EMPTY, UniConstants.slotState.EMPTY));

    //Active motor
    DcMotorEx active;

    //Servo to let balls pass or force into launcher
    Servo ballServo;
    public static double testServoPos = 0;
    private static double servoTarget = 0;

    //Rotary motor and controller
    DcMotorEx rotary;
    public static double pR = 0, dR = 0, lR = 0.1, fR = 0;
    PDFLController rotaryController = new PDFLController(pR, dR, fR, lR);
    public static int rotaryTargetPosition = 0;
    private int rotaryCurrentPosition = 0;
    private int rotaryDirection = 1;
    //Debug rotary
    public static double rotaryPower = 0;

    public servoState state = servoState.INTAKE;

    Timer flipTimer = new Timer();

    public RotaryIntakeSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry, UniConstants.teamColor color){
        this.telemetry = telemetry;
        this.color = color;

        //Active Intake Setup
        active = hardwareMap.get(DcMotorEx.class, UniConstants.ACTIVE_INTAKE_STRING);
        active.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        active.setDirection(UniConstants.ACTIVE_DIRECTION);

        //Ball servo
        ballServo = hardwareMap.get(Servo.class, UniConstants.BALL_SERVO_STRING);


        //Rotary Setup
        rotary = hardwareMap.get(DcMotorEx.class, UniConstants.ROTARY_STRING);
        rotary.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotary.setDirection(DcMotorSimple.Direction.FORWARD);

        //Color Sensors Setup
        colorSensors.addAll(
                List.of(
                        (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_FRONT_STRING)),
                        (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_RIGHT_STRING)),
                        (hardwareMap.get(ColorSensor.class, UniConstants.COLOR_SENSOR_SLOT_LEFT_STRING))
                )
        );

    }

    @Override
    public void periodic() {

        rotaryController.setPDFL(pR, dR, fR, lR);

        //readSlots();
        //rotaryCurrentPosition = rotary.getCurrentPosition();


        //TODO: Needs to be tuned for PDFL
        rotaryController.setTarget(rotaryTargetPosition);
        rotaryController.update(rotaryCurrentPosition);
        rotary.setPower(0);

        active.setPower(isEnabled ? (isReversed ? -1 : 1) : 0);

        if(state == servoState.OUTTAKE){
            active.setPower(1);
        }

        ballServo.setPosition((state == servoState.INTAKE ? UniConstants.SERVO_INTAKE : UniConstants.SERVO_OUTTAKE));

    }

    public void disableActive(){
        isEnabled = false;
    }

    public void enableActive(){
        isEnabled = true;
    }

    public void reverseIntake(){
        isReversed = true;
    }
    public void forwardIntake(){
        isReversed = false;
    }

    public void setRotaryTargetPosition(int target){
        rotaryTargetPosition = target;
    }

    public int getRotaryTargetPosition(){
        return rotaryTargetPosition;
    }

    public int getRotaryCurrentPosition(){
        return rotaryCurrentPosition;
    }

    public int getError(){
        return Math.abs(rotaryTargetPosition - rotaryCurrentPosition);
    }

    public void toggleServo(){
        if(state == servoState.INTAKE){
            state = servoState.OUTTAKE;
        }
        else {
            state = servoState.INTAKE;
        }
    }

    public void toggleServo(servoState testState){
        state = testState;
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

    public boolean isFull(UniConstants.slotState slot) {
        return (slot == UniConstants.slotState.PURPLE) || (slot == UniConstants.slotState.GREEN);
    }

    public void setRotaryPower(double power){
        rotaryPower = power;
    }
    public void setRotaryDirection(int direction){
        rotaryDirection = direction;
    }

    public void setColor(UniConstants.teamColor color){
        this.color = color;
    }

    public boolean allFull() {

        for (UniConstants.slotState slot : slots) {
            if (!isFull(slot)) {
                return false;
            }
        }
        return true;
    }

    public boolean isAtPosition(){
        return getError() <= 5;
    }




    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF ROTARY LOG");
                telemetry.addData("Rotary Current Pos ", rotaryCurrentPosition);
                telemetry.addData("Rotary Target Pos ", rotaryTargetPosition);
                telemetry.addLine();
                telemetry.addData("Slot Front State ", slots.get(0));
                telemetry.addData("Slot Right State ", slots.get(1));
                telemetry.addData("Slot Left State ", slots.get(2));
                telemetry.addLine("END OF ROTARY LOG");
                telemetry.addLine();
                break;
            case EXTREME:
                telemetry.addLine("START OF ROTARY LOG");
                telemetry.addData("Rotary Current Pos ", rotaryCurrentPosition);
                telemetry.addData("Rotary Target Pos ", rotaryTargetPosition);
                telemetry.addData("Rotary Error ", getError());
                telemetry.addData("Rotary Power Debug ", rotaryPower);
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
                telemetry.addLine("END OF ROTARY LOG");


        }
    }

    public enum servoState{
        INTAKE,
        OUTTAKE
    }


}
