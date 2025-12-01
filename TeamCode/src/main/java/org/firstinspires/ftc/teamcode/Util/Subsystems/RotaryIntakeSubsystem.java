package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Util.PDFLController;
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
    Servo flickerServoFront;
    Servo flickerServoRight;
    Servo flickerServoLeft;
    public static double testServoPos = 0;


    //Rotary motor and controller

    //Debug rotary
    public static double rotaryPower = 0;

    public servoState stateFront = servoState.INTAKE;
    public servoState stateRight = servoState.INTAKE;
    public servoState stateLeft = servoState.INTAKE;

    public RotaryIntakeSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry, UniConstants.teamColor color){
        this.telemetry = telemetry;
        this.color = color;

        //Active Intake Setup
        active = hardwareMap.get(DcMotorEx.class, UniConstants.ACTIVE_INTAKE_STRING);
        active.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        active.setDirection(UniConstants.ACTIVE_DIRECTION);

        //Ball servo
        flickerServoFront = hardwareMap.get(Servo.class, UniConstants.FLICKER_FRONT_STRING);
        flickerServoRight = hardwareMap.get(Servo.class, UniConstants.FLICKER_RIGHT_STRING);
        flickerServoLeft = hardwareMap.get(Servo.class, UniConstants.FLICKER_LEFT_STRING);




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



        //readSlots();
        active.setPower(isEnabled ? (isReversed ? -1 : 1) : 0);



        //Flicker Servos
        flickerServoFront.setPosition(debug ? testServoPos : (stateFront == servoState.INTAKE ? UniConstants.FLICKER_DOWN : UniConstants.FLICKER_UP));
        flickerServoRight.setPosition(debug ? testServoPos : (stateRight == servoState.INTAKE ? UniConstants.FLICKER_DOWN : UniConstants.FLICKER_UP));
        flickerServoLeft.setPosition(debug ? testServoPos : (stateLeft == servoState.INTAKE ? UniConstants.FLICKER_DOWN : UniConstants.FLICKER_UP));

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






    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF ROTARY LOG");
                telemetry.addLine();
                telemetry.addData("Slot Front State ", slots.get(0));
                telemetry.addData("Slot Right State ", slots.get(1));
                telemetry.addData("Slot Left State ", slots.get(2));
                telemetry.addLine("END OF ROTARY LOG");
                telemetry.addLine();
                break;
            case EXTREME:
                telemetry.addLine("START OF ROTARY LOG");
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
