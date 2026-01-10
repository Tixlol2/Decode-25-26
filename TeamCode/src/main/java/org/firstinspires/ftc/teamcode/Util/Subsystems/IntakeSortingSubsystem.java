package org.firstinspires.ftc.teamcode.Util.Subsystems;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.IfElseCommand;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class IntakeSortingSubsystem implements Subsystem {
    //Class variables
    JoinedTelemetry telemetry;
    public static boolean debug = false;
    public boolean isEnabled = false;
    public static boolean isReversed = false;



    //Active motor
    MotorEx active = new MotorEx(UniConstants.ACTIVE_INTAKE_STRING).floatMode().reversed();
    public static UniConstants.servoState state = UniConstants.servoState.DOWN;

    public static Slot backSlot;
    public static Slot rightSlot;
    public static Slot leftSlot;

    public ArrayList<Slot> slots;

    public static IntakeSortingSubsystem INSTANCE = new IntakeSortingSubsystem();

    public IntakeSortingSubsystem(){}

    @Override
    public void initialize(){
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());

        backSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_BACK_STRING, UniConstants.COLOR_SENSOR_SLOT_BACK_STRING, UniConstants.LIGHT_BACK_STRING, telemetry);
        rightSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_RIGHT_STRING, UniConstants.COLOR_SENSOR_SLOT_RIGHT_STRING, UniConstants.LIGHT_RIGHT_STRING, telemetry);
        leftSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_LEFT_STRING, UniConstants.COLOR_SENSOR_SLOT_LEFT_STRING, UniConstants.LIGHT_LEFT_STRING, telemetry);

        slots = new ArrayList<>(Arrays.asList(backSlot, rightSlot, leftSlot));
    }


    @Override
    public void periodic() {

        if(ActiveOpMode.isStarted()) {

            backSlot.update();
            rightSlot.update();
            leftSlot.update();

            //if ANY servos are up, set state to OUTTAKE so the active is running
            if (backSlot.getTargetPosition().equals(UniConstants.servoState.DOWN) && rightSlot.getTargetPosition().equals(UniConstants.servoState.DOWN) && leftSlot.getTargetPosition().equals(UniConstants.servoState.DOWN)) {
                state = UniConstants.servoState.DOWN;
            } else {
                state = UniConstants.servoState.UP;
            }


            active.setPower(isEnabled ? (isReversed ? -1 : 1) : 0);

        }

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
        enableActive();
    }

    public void forwardIntake(){
        isReversed = false;
        enableActive();
    }
    public boolean shouldRumble(){return allFull();}

    public Command runActive(){
        return new InstantCommand(() -> {forwardIntake(); enableActive();});
    }
    public Command stopActive(){
        return new InstantCommand(this::disableActive);
    }

    public Command setServoState(Slot slot, UniConstants.servoState state) {
        return new InstantCommand(() -> {slot.setTargetPosition(state);}).named(slot.name + " Set Servo State");
    }

    public Command Shoot(Slot slot, boolean isLast){
        return new SequentialGroup(
                setServoState(slot, UniConstants.servoState.UP),
                new Delay(UniConstants.FAST_FLICKER_TIME_UP),
                setServoState(slot, UniConstants.servoState.DOWN),
                new IfElseCommand(() -> isLast, new NullCommand(), new Delay(UniConstants.FAST_FLICKER_TIME_DOWN))
        );
    }



//    public Command launchInPattern(Slot first, Slot second, Slot third){
//        return new SequentialGroup(
//                setServoState(first, UniConstants.servoState.UP),
//                new Delay(UniConstants.FAST_FLICKER_TIME_UP),
//                setServoState(first, UniConstants.servoState.DOWN),
//                new Delay(UniConstants.FAST_FLICKER_TIME_DOWN),
//                setServoState(second, UniConstants.servoState.UP),
//                new Delay(UniConstants.FAST_FLICKER_TIME_UP),
//                setServoState(second, UniConstants.servoState.DOWN),
//                new Delay(UniConstants.FAST_FLICKER_TIME_DOWN),
//                setServoState(third, UniConstants.servoState.UP),
//                new Delay(UniConstants.FAST_FLICKER_TIME_UP),
//                setServoState(third, UniConstants.servoState.DOWN)
//        );
//    }


    public int getGreenCount(){
        int greens = 0;
        for(Slot slot : slots){
            if(slot.colorState.equals(UniConstants.slotState.GREEN)){
                greens++;
            }
        }
        return greens;
    }

    public int getPurpleCount(){
        int purples = 0;
        for(Slot slot : slots){
            if(slot.colorState.equals(UniConstants.slotState.PURPLE)){
                purples++;
            }
        }
        return purples;
    }

    //BANGGGGGGGGGGG
    public ArrayList<Slot> determineOrder(@NonNull ArrayList<UniConstants.slotState> pattern) {
        ArrayList<Slot> result = new ArrayList<>();
        Set<Slot> used = new HashSet<>();

        for (UniConstants.slotState wanted : pattern) {
            for (Slot slot : slots) {
                if (slot.colorState == wanted && used.add(slot)) {
                    result.add(slot);
                    break;
                }
            }
        }

        if(result.size() < 3){
            return new ArrayList<>(Arrays.asList(backSlot, rightSlot, leftSlot));
        }

        return result;
    }


    public boolean allFull() {
        return backSlot.isFull() && rightSlot.isFull() && leftSlot.isFull();
    }



    public void sendSlotTelemetry(UniConstants.loggingState state){
        backSlot.sendTelemetry(state);
        rightSlot.sendTelemetry(state);
        leftSlot.sendTelemetry(state);
    }

    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF SORTING LOG");
                telemetry.addData("Purple Count: ", getPurpleCount());
                telemetry.addData("Green Count: ", getGreenCount());
                telemetry.addData("Intake Enabled ", isEnabled);
                telemetry.addData("Intake Reversed ", isReversed);
                telemetry.addData("All Full ", allFull());
                sendSlotTelemetry(state);
                telemetry.addLine("END OF SORTING LOG");
                telemetry.addLine();
                break;
            case EXTREME:
                telemetry.addLine("START OF ROTARY LOG");
                telemetry.addLine("END OF ROTARY LOG");


        }
    }

    public void setTelemetry(JoinedTelemetry telemetry){
        this.telemetry = telemetry;
    }


    public static class Slot {

        private final ServoEx kickerServo;
        private final ServoEx light;
        private final ColorSensor colorSensor;

        String name = "";

        private UniConstants.slotState colorState = UniConstants.slotState.EMPTY;
        private UniConstants.servoState servoState = UniConstants.servoState.DOWN;
        private JoinedTelemetry telemetry;

        double up = 0;
        double down = 0;

        public Slot(HardwareMap hardwareMap, String kickerServoName, String colorSensorName, String lightName, JoinedTelemetry telemetry){

            kickerServo = new ServoEx(kickerServoName);
            light = new ServoEx(lightName);
            name = kickerServoName;
            colorSensor = hardwareMap.get(ColorSensor.class, colorSensorName);
            Timer readTimer = new Timer();

            this.telemetry = telemetry;

            //Identify slot and assign servo constants
            if(name.equals(UniConstants.FLICKER_BACK_STRING)){
                up = UniConstants.FLICKER_BACK_UP; //Back up
                down = UniConstants.FLICKER_BACK_DOWN; //Back down
            } else if (name.equals(UniConstants.FLICKER_LEFT_STRING)) {
                up = UniConstants.FLICKER_LEFT_UP; //Left up
                down = UniConstants.FLICKER_LEFT_DOWN; //Left down
            } else {
                up = UniConstants.FLICKER_RIGHT_UP; //Right up
                down = UniConstants.FLICKER_RIGHT_DOWN; //Right down
            }



        }

        public void update(){

//            //ONLY UNCOMMENT FOR TESTING
//            if(name.equals(UniConstants.FLICKER_BACK_STRING)){
//                up = UniConstants.FLICKER_BACK_UP; //Back up
//                down = UniConstants.FLICKER_BACK_DOWN; //Back down
//            } else if (name.equals(UniConstants.FLICKER_LEFT_STRING)) {
//                up = UniConstants.FLICKER_LEFT_UP; //Left up
//                down = UniConstants.FLICKER_LEFT_DOWN; //Left down
//            } else {
//                up = UniConstants.FLICKER_RIGHT_UP; //Right up
//                down = UniConstants.FLICKER_RIGHT_DOWN; //Right down
//            }

            //Update Colors
            readSlot();

            //Update Servo
            kickerServo.setPosition(state == UniConstants.servoState.UP ? ((servoState == UniConstants.servoState.UP) ? up : down) : down);
            light.setPosition(isFull() ? (colorState.equals(UniConstants.slotState.GREEN) ? .5 : .722) : 0);
        }

        public String getName(){
            return name;
        }

        private void readSlot() {

            double red = colorSensor.red();
            double green = colorSensor.green();
            double blue = colorSensor.blue();
            double alpha = colorSensor.alpha();
            if (((green > red) && (blue > red)) && (alpha < 5000) && green > 55) {
                colorState = UniConstants.slotState.GREEN;
            } else if (((red > green) && (blue > green)) && (alpha < 5000)) {
                colorState = UniConstants.slotState.PURPLE;
            } else{
                colorState = UniConstants.slotState.EMPTY;
            }


        }

        public UniConstants.slotState getColorState(){
            return colorState;
        }

        public UniConstants.servoState getTargetPosition(){
            return servoState;
        }

        public void setTargetPosition(UniConstants.servoState state){
            servoState = state;
        }

        public boolean isFull() {
            return (colorState == UniConstants.slotState.PURPLE) || (colorState == UniConstants.slotState.GREEN);
        }

        public void sendTelemetry(UniConstants.loggingState state){
            switch(state){
                case DISABLED:
                    break;
                case ENABLED:
                    telemetry.addData("Name ", name);
                    telemetry.addData("Color State ", colorState);
                    telemetry.addLine();
                    break;
                case EXTREME:
                    telemetry.addLine("START OF SLOT LOG");
                    telemetry.addData("Name ", name);
                    telemetry.addData("Kicker Up ", up);
                    telemetry.addData("Kicker Down ", down);
                    telemetry.addData("Color State ", colorState);
                    telemetry.addData("Red ", colorSensor.red());
                    telemetry.addData("Green ", colorSensor.green());
                    telemetry.addData("Blue ", colorSensor.blue());
                    telemetry.addLine();
                    telemetry.addData("Is Full ", isFull());
                    telemetry.addLine("END OF SLOT LOG");
                    telemetry.addLine();
                    break;

            }







        }


        public void setTelemetry(JoinedTelemetry telemetry){
            this.telemetry = telemetry;
        }


    }
}