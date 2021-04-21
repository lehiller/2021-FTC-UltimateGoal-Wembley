package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

public class DriveTrain extends Subsystem {


    private DcMotorEx lfDrive, lmDrive,lbDrive, rfDrive, rmDrive, rbDrive;
    private DcMotorEx[] motors;
    private boolean isArcadeDrive;
    public DriveTrain()
    {
        super("DriveTrain");
        lfDrive = Robot.hw.get(DcMotorEx.class, RobotMap.LEFT_FRONT_DRIVE_ID);
        lbDrive = Robot.hw.get(DcMotorEx.class, RobotMap.LEFT_BACK_DRIVE_ID);
        rfDrive = Robot.hw.get(DcMotorEx.class, RobotMap.RIGHT_FRONT_DRIVE_ID);
        rbDrive = Robot.hw.get(DcMotorEx.class, RobotMap.RIGHT_BACK_DRIVE_ID);

        lfDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        lbDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        rfDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rbDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotorEx[4];

        motors[0] = lfDrive;
        motors[1] = lbDrive;
        motors[2] = rfDrive;
        motors[3] = rbDrive;

        isArcadeDrive = false;
    }

    public void setBrakeMode(boolean brakeModeOn)
    {
        for(DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(brakeModeOn ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void move(double lSpeed, double rSpeed)
    {
        move(lSpeed, rSpeed, true);
    }

    public void move(double lSpeed, double rSpeed, boolean squareMovement)
    {
        lSpeed = Range.clip(lSpeed, -1, 1);
        rSpeed = Range.clip(rSpeed, -1, 1);

        // The x^2 movement Dillan wanted.
        if(squareMovement) {
            lSpeed *= Math.abs(lSpeed);
            rSpeed *= Math.abs(rSpeed);
        }

//        lSpeed *= Constants.DRIVE_BUFFER;
//        rSpeed *= Constants.DRIVE_BUFFER;



        for(int i = 0; i < motors.length; i++)
            if(i < motors.length/2)
                motors[i].setPower(lSpeed);
            else
                motors[i].setPower(rSpeed);
    }

    public void tankDrive()
    {
        move(-Robot.g1.left_stick_y, -Robot.g1.right_stick_y);
    }

    public void arcadeDrive()
    {
        move( -(Robot.g1.right_stick_y - Robot.g1.left_stick_x), -(Robot.g1.right_stick_y + Robot.g1.left_stick_x));
    }
    @Override
    public void run() {
        if(isArcadeDrive)
            arcadeDrive();
        else
            tankDrive();

        if(Robot.g1.dpad_up)
            isArcadeDrive = false;
        else if(Robot.g1.dpad_down)
            isArcadeDrive = true;
    }

    public void test() {
        rmDrive.setPower(0.2);
    }

    @Override
    public void stop() {
        move(0,0);
    }
}




