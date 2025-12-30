package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import java.util.ArrayList;
import java.util.Arrays;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class IntakeSortingSubsystem implements Subsystem {
    //Class variables
    JoinedTelemetry telemetry;
    public static boolean debug = false;
    public static boolean isEnabled = false;
    public static boolean isReversed = false;






    //Active motor
    MotorEx active = new MotorEx(UniConstants.ACTIVE_INTAKE_STRING).floatMode().reversed();
    public static UniConstants.servoState state = UniConstants.servoState.DOWN;

    public static Slot backSlot;
    public static Slot rightSlot;
    public static Slot leftSlot;

    public ArrayList<Slot> slots;

    public IntakeSortingSubsystem(){}

    @Override
    public void initialize(){
        telemetry = new JoinedTelemetry(ActiveOpMode.telemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());

        backSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_BACK_STRING, UniConstants.COLOR_SENSOR_SLOT_BACK_STRING, telemetry);
        rightSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_RIGHT_STRING, UniConstants.COLOR_SENSOR_SLOT_RIGHT_STRING, telemetry);
        leftSlot = new Slot(ActiveOpMode.hardwareMap(), UniConstants.FLICKER_LEFT_STRING, UniConstants.COLOR_SENSOR_SLOT_LEFT_STRING, telemetry);

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
    }

    public void forwardIntake(){
        isReversed = false;
    }
    public boolean shouldRumble(){return allFull();}

    //Command Testing
    public Command setServoState(Slot slot, UniConstants.servoState state){
        return new LambdaCommand()
                .setStart(() -> {
                    // Runs on start
                    slot.setTargetPosition(state);

                })
                .setUpdate(() -> {
                    // Runs on update
                })
                .setStop(interrupted -> {
                    // Runs on stop

                })
                .setIsDone(() -> true) // Returns if the command has finished
                .requires(this)
                .setInterruptible(false)
                .named("Set Servo State"); // sets the name of the command; optional
    }



    public Command launchInPattern(Slot first, Slot second, Slot third){
        return new SequentialGroup(
                setServoState(first, UniConstants.servoState.UP),
                new Delay(UniConstants.TIME_BETWEEN_SHOT_SECONDS / 2),
                setServoState(first, UniConstants.servoState.DOWN),
                new Delay(UniConstants.TIME_BETWEEN_SHOT_SECONDS / 2),
                setServoState(second, UniConstants.servoState.UP),
                new Delay(UniConstants.TIME_BETWEEN_SHOT_SECONDS / 2),
                setServoState(second, UniConstants.servoState.DOWN),
                new Delay(UniConstants.TIME_BETWEEN_SHOT_SECONDS / 2),
                setServoState(third, UniConstants.servoState.UP),
                new Delay(UniConstants.TIME_BETWEEN_SHOT_SECONDS / 2),
                setServoState(third, UniConstants.servoState.DOWN)
        );
    }


    //BANGGGGGGGGGGG
    public Command shoot(ArrayList<UniConstants.slotState> pattern){
        Slot first = null;
        Slot second = null;
        Slot third = null;

        boolean[] slotUsed = new boolean[3]; // which slots we've already queued up

        // try to match what the pattern wants
        if(pattern != null){
            int patternSize = pattern.size();
            for(int p = 0; p < patternSize; p++){
                UniConstants.slotState desiredColor = pattern.get(p);

                // look for a slot with this color that we haven't used yet
                for(int s = 0; s < 3; s++){
                    if(!slotUsed[s] && slots.get(s).getColorState().equals(desiredColor)){
                        slotUsed[s] = true;

                        // assign based on which position in pattern we're at
                        if(p == 0) first = slots.get(s);
                        else if(p == 1) second = slots.get(s);
                        else if(p == 2) third = slots.get(s);

                        break;
                    }
                }
            }
        }

        // if we couldn't fill all 3 positions from the pattern, grab any full slots we have left
        // doesn't matter what order at this point, just shoot what we got
        if(first == null || second == null || third == null){
            int nextSlotIndex = 0;

            if(first == null){
                while(nextSlotIndex < 3 && (slotUsed[nextSlotIndex] || !slots.get(nextSlotIndex).isFull())) nextSlotIndex++;
                if(nextSlotIndex < 3){
                    first = slots.get(nextSlotIndex);
                    slotUsed[nextSlotIndex] = true;
                }
            }

            if(second == null){
                nextSlotIndex = 0;
                while(nextSlotIndex < 3 && (slotUsed[nextSlotIndex] || !slots.get(nextSlotIndex).isFull())) nextSlotIndex++;
                if(nextSlotIndex < 3){
                    second = slots.get(nextSlotIndex);
                    slotUsed[nextSlotIndex] = true;
                }
            }

            if(third == null){
                nextSlotIndex = 0;
                while(nextSlotIndex < 3 && (slotUsed[nextSlotIndex] || !slots.get(nextSlotIndex).isFull())) nextSlotIndex++;
                if(nextSlotIndex < 3){
                    third = slots.get(nextSlotIndex);
                    slotUsed[nextSlotIndex] = true;
                }
            }
        }

        // still have nulls? just use defaults but make sure we don't double-shoot anything
        if(first == null){
            if(backSlot != second && backSlot != third) first = backSlot;
            else if(rightSlot != second && rightSlot != third) first = rightSlot;
            else first = leftSlot;
        }
        if(second == null){
            if(rightSlot != first && rightSlot != third) second = rightSlot;
            else if(backSlot != first && backSlot != third) second = backSlot;
            else second = leftSlot;
        }
        if(third == null){
            if(leftSlot != first && leftSlot != second) third = leftSlot;
            else if(rightSlot != first && rightSlot != second) third = rightSlot;
            else third = backSlot;
        }

        return launchInPattern(first, second, third);
    }

    public boolean allFull() {
        return backSlot.isFull() && rightSlot.isFull() && leftSlot.isFull();
    }


    public void sendTelemetry(UniConstants.loggingState state){
        switch(state){
            case DISABLED:
                break;
            case ENABLED:
                telemetry.addLine("START OF SORTING LOG");
                telemetry.addData("Intake Enabled ", isEnabled);
                telemetry.addData("Intake Reversed ", isReversed);
                telemetry.addData("All Full ", allFull());
                telemetry.addLine("END OF SORTING LOG");
                telemetry.addLine();
                break;
            case EXTREME:
                telemetry.addLine("START OF ROTARY LOG");
                telemetry.addLine("END OF ROTARY LOG");


        }
    }


    public static class Slot {

        private final ServoEx kickerServo;
        private final ColorSensor colorSensor;

        String name = "";

        private UniConstants.slotState colorState = UniConstants.slotState.EMPTY;
        private UniConstants.servoState servoState = UniConstants.servoState.DOWN;
        private final JoinedTelemetry telemetry;

        double up = 0;
        double down = 0;

        public Slot(HardwareMap hardwareMap, String kickerServoName, String colorSensorName, JoinedTelemetry telemetry){

            kickerServo = new ServoEx(kickerServoName);
            name = kickerServoName;
            colorSensor = hardwareMap.get(ColorSensor.class, colorSensorName);

            this.telemetry = telemetry;


            if(kickerServoName.equals(UniConstants.FLICKER_BACK_STRING)){
                up = .6; //Back up
                down = .15; //Back down
            } else if (kickerServoName.equals(UniConstants.FLICKER_LEFT_STRING)) {
                up = .85; //Left up
                down = .4; //Left down
            } else {
                up = .5; //Right up
                down = .9; //Right down
            }



        }

        public void update(){

            //Update Colors
            readSlot();

            //Update Servo
            kickerServo.setPosition(state == UniConstants.servoState.UP ? ((servoState == UniConstants.servoState.UP) ? up : down) : down);
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
            } else {
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
                    telemetry.addLine("START OF SLOT LOG");
                    telemetry.addData("Name ", name);
                    telemetry.addData("Kicker Up ", up);
                    telemetry.addData("Kicker Down ", down);
                    telemetry.addData("Color State ", colorState);
                    telemetry.addData("Is Full ", isFull());
                    telemetry.addLine("END OF SLOT LOG");
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



    }
}
