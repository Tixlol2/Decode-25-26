package org.firstinspires.ftc.teamcode.Subsystems.Slots;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.UniConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

public class MainSlot implements Subsystem {

    private final Timer shotTimer = new Timer();
    private ServoEx light;
    private final String lightName;
    private ColorSensor colorSensor;
    private final String colorSensorName;
    private ServoEx kickerServo;
    private final String kickerName;
    private double up = 0;
    private double down = 0;
    private SlotState colorState = SlotState.EMPTY;
    private ServoState servoState = ServoState.DOWN;

    public MainSlot(String lightName, String colorSensorName, String kickerName, double up, double down) {
        this.lightName = lightName;
        this.colorSensorName = colorSensorName;
        this.kickerName = kickerName;
        this.up = up;
        this.down = down;
    }



    @Override
    public void initialize() {
        kickerServo = new ServoEx(kickerName);
        light = new ServoEx(lightName);
        colorSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, colorSensorName);

    }

    @Override
    public void periodic() {
        //Update Colors
        readSlot();

        //Update Servo
        kickerServo.setPosition((servoState == ServoState.UP) ? up : down);
        light.setPosition(isFull() ? (colorState.equals(SlotState.GREEN) ? .5 : .722) : 0);
    }

    public String getKickerServoName() {
        return kickerName;
    }

    public SlotState getColorState() {
        return colorState;
    }


    public Command basicShoot() {
        return new SequentialGroup(
                setServoState(ServoState.UP),
                new Delay(UniConstants.FAST_FLICKER_TIME_UP),
                setServoState(ServoState.DOWN)

        ).requires("Shooting");
    }

    public Command basicShootDown() {
        return new SequentialGroup(
                setServoState(ServoState.UP),
                new Delay(UniConstants.FAST_FLICKER_TIME_UP),
                setServoState(ServoState.DOWN),
                new Delay(UniConstants.FAST_FLICKER_TIME_DOWN)
        ).requires("Shooting");
    }

    public Command setServoState(ServoState state) {
        return new InstantCommand(() -> servoState = state);
    }


    public  boolean isFull() {
        return (colorState == SlotState.PURPLE) || (colorState == SlotState.GREEN);
    }

    public void readSlot() {
        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();
        double alpha = colorSensor.alpha();


        if (((green > red) && (blue > red)) && (alpha < 5000) && green > 65) {
            colorState = SlotState.GREEN;
        } else if (((red > green) && (blue > green)) && (alpha < 5000)) {
            colorState = SlotState.PURPLE;
        } else {
            colorState = SlotState.EMPTY;
        }
    }


    public enum SlotState {
        PURPLE,
        GREEN,
        EMPTY
    }

    public enum ServoState {
        DOWN,
        UP
    }
}
