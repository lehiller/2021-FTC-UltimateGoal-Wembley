package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;


public class RingGrabber extends Subsystem {

    private CRServo Finger;
    public RingGrabber()
    {
        super("RingGrabber");
        Finger = Robot.hw.get(CRServo.class, RobotMap.FINGER_ID);
    }

    public void clockwise()
    {
        move(Constants.INTAKE_SPEED);
    }

    public void counterclockwise()
    {
        move(-Constants.INTAKE_SPEED);
    }

    public void move(double speed) {
        Finger.setPower(speed);
    }

    @Override
    public void run() {
        if(Robot.g2.right_bumper)
            clockwise();
        else if(Robot.g2.left_bumper)
            counterclockwise();
        else
            stop();
    }


    //Auto Methods for Ring Grabber

    //don't exist yet...





    @Override
    public void stop() {
        move(0);
    }
}
