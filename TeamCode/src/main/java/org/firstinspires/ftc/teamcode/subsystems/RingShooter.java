package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
import org.firstinspires.ftc.teamcode.util.Constants;

public class RingShooter extends Subsystem {

    private DcMotorEx LeftOut, RightOut;

    private CRServo Trigger, Bump;


    public RingShooter()
    {
        super("RingShooter");
        LeftOut = Robot.hw.get(DcMotorEx.class, RobotMap.LEFT_OUT_ID);
        RightOut = Robot.hw.get(DcMotorEx.class, RobotMap.RIGHT_OUT_ID);

        LeftOut.setDirection(DcMotorSimple.Direction.REVERSE);
        RightOut.setDirection(DcMotorSimple.Direction.REVERSE);

        Trigger = Robot.hw.get(CRServo.class, RobotMap.TRIGGER_ID);
        Bump = Robot.hw.get(CRServo.class, RobotMap.BUMP_ID);



    }



    public void  spin(double speed)
    {
        RightOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightOut.setVelocity(speed);
        LeftOut.setVelocity(speed);
    }




    public void spinup()
    {
        spin (Constants.HIGH_SPIN_RATE);
    }


    public void powershot()
    {
        spin( Constants.POWER_SPIN_RATE);

    }








    @Override
    public void run() {
        if (Robot.g2.y)
            spinup();
        else
            stop();

        if (Robot.g2.start) {
            Bump.setPower(1);
        }
        else if(Robot.g2.back) {
            Bump.setPower(-1);
        }
        else {
            Bump.setPower(0);
        }

        if (Robot.g2.a) {
            Trigger.setPower(1);
        }
        else if(Robot.g2.b) {
            Trigger.setPower(-1);
        }
        else {
                Trigger.setPower(0);
        }


    }

    @Override
    public void stop(){

        spin (0);


    }

}



