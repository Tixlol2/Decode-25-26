package org.firstinspires.ftc.teamcode.OpModes.Paths;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;

public class Close6 extends MainPaths {

    public Close6() {
        super("Close 6", 4, ShootingLocation.SHORT, Robot.color == Robot.teamColor.BLUE ? Poses.blueGoalTopStartFacing : Poses.redGoalTopStartFacing);
        init();
    }

    public void init() {
        setCommand(1, MoveShoot());
        setCommand(2, IntakeTop());
        setCommand(3, MoveShoot());
        setCommand(4, Park());
    }


}
