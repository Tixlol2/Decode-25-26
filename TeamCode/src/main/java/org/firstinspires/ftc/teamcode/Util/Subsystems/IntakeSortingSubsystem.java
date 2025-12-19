package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Util.UniConstants;


import java.util.ArrayList;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class IntakeSortingSubsystem implements Subsystem {
    //Class variables
    JoinedTelemetry telemetry;
    public static boolean debug = false;
    public static boolean isEnabled = false;
    public static boolean isReversed = false;


    public ArrayList<ColorSensor> colorSensors = new ArrayList<>();
    //Front Left Right
    public ArrayList<UniConstants.slotState> slots = new ArrayList<>(List.of(UniConstants.slotState.EMPTY, UniConstants.slotState.EMPTY, UniConstants.slotState.EMPTY));

    //Active motor
    MotorEx active = new MotorEx(UniConstants.ACTIVE_INTAKE_STRING).floatMode().reversed();
    public servoState state = servoState.INTAKE;

    //Servo Init
    public ServoEx frontServo = new ServoEx(UniConstants.FLICKER_FRONT_STRING);
    public ServoEx rightServo = new ServoEx(UniConstants.FLICKER_RIGHT_STRING);
    public ServoEx leftServo = new ServoEx(UniConstants.FLICKER_LEFT_STRING);

    public ArrayList<ServoEx> servos = new ArrayList<>(List.of(frontServo, leftServo, rightServo));


    double frontTarget = UniConstants.FLICKER_DOWN, rightTarget = UniConstants.FLICKER_DOWN, leftTarget = UniConstants.FLICKER_DOWN;



    public IntakeSortingSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry){
        this.telemetry = telemetry;
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

        //if ANY servos are up, set state to OUTTAKE so the active is running
        //TODO: Make sure the flicker up and down are the right values for all servos
        if(frontTarget != UniConstants.FLICKER_UP && rightTarget != UniConstants.FLICKER_UP && leftTarget != UniConstants.FLICKER_UP){
            state = servoState.INTAKE;
        } else{
            state = servoState.OUTTAKE;
        }

        readSlots();
        active.setPower(isEnabled ? (isReversed ? -1 : 1) : 0);

        if(state == servoState.OUTTAKE){
            active.setPower(1);
        }

        frontServo.setPosition(frontTarget);
        leftServo.setPosition(leftTarget);
        rightServo.setPosition(rightTarget);

    }

    //Util
    public enum servoState{
        INTAKE,
        OUTTAKE
    }



    //Active Methods
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

    //Command Testing
    public Command setServoState(ServoEx servo, double target){
        return new LambdaCommand()
                .setStart(() -> {
                    // Runs on start
                    servo.setPosition(target);

                })
                .setUpdate(() -> {
                    // Runs on update
                })
                .setStop(interrupted -> {
                    // Runs on stop
                    servo.setPosition(UniConstants.FLICKER_DOWN);
                })
                .setIsDone(() -> true) // Returns if the command has finished
                .requires(this)
                .setInterruptible(false)
                .named("Set Servo State"); // sets the name of the command; optional
    }

    public Command launchInPattern(ServoEx firstServo, ServoEx secondServo, ServoEx thirdServo){
        return new SequentialGroup(
                setServoState(firstServo, UniConstants.FLICKER_UP),
                new Delay(.25),
                setServoState(secondServo, UniConstants.FLICKER_UP),
                new Delay(.25),
                setServoState(thirdServo, UniConstants.FLICKER_UP)
        );
    }


    //Color Sensor Methods
    //TODO: god i have to refine this cause it lowkey dont work
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
                telemetry.addLine("START OF SORTING LOG");
                telemetry.addData("Intake Enabled ", isEnabled);
                telemetry.addData("Intake Reversed ", isReversed);
                telemetry.addLine();
                telemetry.addData("Slot Front State ", slots.get(0));
                telemetry.addData("Slot Right State ", slots.get(1));
                telemetry.addData("Slot Left State ", slots.get(2));
                telemetry.addLine("END OF SORTING LOG");
                telemetry.addLine();
                break;
            case EXTREME:
                telemetry.addLine("START OF ROTARY LOG");
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




}
