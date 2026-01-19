package org.firstinspires.ftc.teamcode.Util.Subsystems.Slots;

import org.firstinspires.ftc.teamcode.Util.UniConstants;

public class BackSlot extends Slot {

    public static final BackSlot INSTANCE = new BackSlot();

    public BackSlot() {
        super(UniConstants.LIGHT_BACK_STRING, UniConstants.COLOR_SENSOR_SLOT_BACK_STRING, UniConstants.FLICKER_BACK_STRING, UniConstants.FLICKER_BACK_UP, UniConstants.FLICKER_BACK_DOWN);
    }


}