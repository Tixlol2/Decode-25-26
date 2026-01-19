package org.firstinspires.ftc.teamcode.OpModes.Paths;

import org.firstinspires.ftc.teamcode.Util.Poses;
import org.firstinspires.ftc.teamcode.Util.Subsystems.Robot;

public class Close9 extends MainPaths {

    public Close9() {
        super("Close 9", 6, ShootingLocation.SHORT, Robot.color == Robot.teamColor.BLUE ? Poses.blueGoalTopStartFacing : Poses.redGoalTopStartFacing);
        init();
    }

    public void init() {
        setCommand(1, MoveShoot());
        setCommand(2, IntakeTop());
        setCommand(3, MoveShoot());
        setCommand(4, IntakeMid());
        setCommand(5, MoveShoot());
        setCommand(6, Park());
    }


}
