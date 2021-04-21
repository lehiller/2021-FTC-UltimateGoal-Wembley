package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

public class WobbleGrabber extends Subsystem {

    private CRServo Claw;

    public WobbleGrabber()
    {
        super("WobbleGrabber");
        Claw = Robot.hw.get(CRServo.class, RobotMap.CLAW_ID);
    }


    @Override
    public void run() {
        if(Robot.g2.right_bumper)
            Claw.setPower(-1);
        else if(Robot.g1.left_bumper)
            Claw.setPower(1);
        else
            Claw.setPower(0);
    }

    @Override
    public void stop() {
        Claw.setPower(0);
    }
}
