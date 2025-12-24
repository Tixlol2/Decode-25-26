package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.hardware.impl.ServoEx;

public class Slot {

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


        if(kickerServoName.equals("FBS")){
            up = .6;
            down = .15;
        } else if (kickerServoName.equals("FLS")) {
            up = .85;
            down = .45;
        } else {
            up = .5;
            down = .9;
        }



    }

    public void update(){

        //Update Colors
        readSlot();

        //Update Servo
        kickerServo.setPosition(IntakeSortingSubsystem.state == UniConstants.servoState.UP ? ((servoState == UniConstants.servoState.UP) ? up : down) : down);
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
