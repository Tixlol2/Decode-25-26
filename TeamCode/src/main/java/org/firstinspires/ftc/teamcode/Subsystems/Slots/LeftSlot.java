package org.firstinspires.ftc.teamcode.Subsystems.Slots;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

public class LeftSlot extends MainSlot {

    public static final LeftSlot INSTANCE = new LeftSlot();

    public LeftSlot() {
        super(UniConstants.LIGHT_LEFT_STRING, UniConstants.COLOR_SENSOR_SLOT_LEFT_STRING, UniConstants.FLICKER_LEFT_STRING, UniConstants.FLICKER_LEFT_UP, UniConstants.FLICKER_LEFT_DOWN);
    }


}