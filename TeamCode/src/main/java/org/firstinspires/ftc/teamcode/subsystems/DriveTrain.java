package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

import org.firstinspires.ftc.teamcode.autos.MultiTask;


public class DriveTrain extends Subsystem {



    private DcMotorEx lfDrive,lbDrive, rfDrive,  rbDrive;
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
        Robot.telemetry.addData("Drivemode = " , isArcadeDrive ? "arcade" : "tank");
    }


    public void arcadeDrive()
    {
        move( -(Robot.g1.right_stick_y - Robot.g1.left_stick_x), -(Robot.g1.right_stick_y + Robot.g1.left_stick_x));
        Robot.telemetry.addData("Drivemode = " , isArcadeDrive ? "arcade" : "tank");
    }

    //this sets the drivetrains mode for the driver controller.
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




    //Auto Stuff _________________________

    /**
     * <u>Auto Method</u>
     * Uses the built in PID encoder movements of the DcMotor class to drive to position
     * @param speed - the speed to drive at
     * @param leftInches - the number of inches for the left side to move
     * @param rightInches - the number of inches for the right side to move
     * @param timeoutS - max seconds for the method to run
     * @param f - another robot functionality to run while this loop runs
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS, MultiTask f)
    {
        int lmTarget, lbTarget, rmTarget, rbTarget;

        // Ensure that the opmode is still active
        if (Robot.opMode.opModeIsActive()) {
            resetEncoders();

            // Determine new target position, and pass to motor controller
            lmTarget = lfDrive.getCurrentPosition() + (int)(leftInches * Constants.COUNTS_PER_INCH);
            lbTarget = lbDrive.getCurrentPosition() + (int)(leftInches * Constants.COUNTS_PER_INCH);
            rmTarget = rfDrive.getCurrentPosition() + (int)(rightInches * Constants.COUNTS_PER_INCH);
            rbTarget = rbDrive.getCurrentPosition() + (int)(rightInches * Constants.COUNTS_PER_INCH);
            lfDrive.setTargetPosition(lmTarget);
            lbDrive.setTargetPosition(lbTarget);
            rfDrive.setTargetPosition(rmTarget);
            rbDrive.setTargetPosition(rbTarget);

            // Turn On RUN_TO_POSITION
            lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            Robot.timer.reset();
            lfDrive.setPower(Math.abs(speed));
            lbDrive.setPower(Math.abs(speed));
            rfDrive.setPower(Math.abs(speed));
            rbDrive.setPower(Math.abs(speed));

            //Note: Possibly to make this better, make each side &&
            while (Robot.opMode.opModeIsActive() && (Robot.timer.seconds() < timeoutS) &&
                    ((lfDrive.isBusy() || lbDrive.isBusy()) &&
                            (rfDrive.isBusy() || rbDrive.isBusy()))) {
                f.execute();
            }
            f.stop();
            // Stop all motion;
            lfDrive.setPower(Math.abs(0));
            lbDrive.setPower(Math.abs(0));
            rfDrive.setPower(Math.abs(0));
            rbDrive.setPower(Math.abs(0));

            // Turn off RUN_TO_POSITION
            lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS){
        MultiTask empty = new MultiTask() {
            @Override
            public void execute() {

            }

            @Override
            public void stop() {

            }
        };

        encoderDrive(speed, leftInches, rightInches,timeoutS, empty);


    }


    //The difference between this and encoderDrive is that this one keeps running until both sides of drivetrain
    //have finished moving, not just one
    public void firmEncoderDrive(double speed, double leftInches, double rightInches, double timeoutS, MultiTask f)
    {
        int lmTarget, lbTarget, rmTarget, rbTarget;

        // Ensure that the opmode is still active
        if (Robot.opMode.opModeIsActive()) {
            resetEncoders();
            // Determine new target position, and pass to motor controller
            lmTarget = lfDrive.getCurrentPosition() + (int)(leftInches * Constants.COUNTS_PER_INCH);
            lbTarget = lbDrive.getCurrentPosition() + (int)(leftInches * Constants.COUNTS_PER_INCH);
            rmTarget = rfDrive.getCurrentPosition() + (int)(rightInches * Constants.COUNTS_PER_INCH);
            rbTarget = rbDrive.getCurrentPosition() + (int)(rightInches * Constants.COUNTS_PER_INCH);
            lfDrive.setTargetPosition(lmTarget);
            lbDrive.setTargetPosition(lbTarget);
            rfDrive.setTargetPosition(rmTarget);
            rbDrive.setTargetPosition(rbTarget);

            // Turn On RUN_TO_POSITION
            lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            Robot.timer.reset();
            lfDrive.setPower(Math.abs(speed));
            lbDrive.setPower(Math.abs(speed));
            rfDrive.setPower(Math.abs(speed));
            rbDrive.setPower(Math.abs(speed));

            // the or after the lbdrive, is the difference between this and encoderDrive
            while (Robot.opMode.opModeIsActive() && (Robot.timer.seconds() < timeoutS) &&
                    ((lfDrive.isBusy() || lbDrive.isBusy()) ||
                            (rfDrive.isBusy() || rbDrive.isBusy()))) {

                f.execute();
                printEncodersInInches();
                printEncoders();
                Robot.telemetry.update();
                Robot.gyro.testPrint();
            }
            f.stop();
            // Stop all motion;
            lfDrive.setPower(Math.abs(0));
            lbDrive.setPower(Math.abs(0));
            rfDrive.setPower(Math.abs(0));
            rbDrive.setPower(Math.abs(0));

            // Turn off RUN_TO_POSITION
            lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.gyro.testPrint();
        }
    }





    /**
     * <u>Auto Method</u>
     * Method tries to have the robot drive and turn at the same time, by running the sides at different speeds
     * @param leftSpeed - the speed for the left side
     * @param rightSpeed - the speed for the right side
     * @param desiredHeading - the heading to finish turning at
     * @param timeoutS - the max amount of time to run the loop
     * @param f - another robot functionality to run while this loop runs
     */

    public void driveAndTurn(double leftSpeed, double rightSpeed, double desiredHeading, double timeoutS, MultiTask f)
    {
        if(Robot.opMode.opModeIsActive()) {
            lfDrive.setPower(leftSpeed);
            lbDrive.setPower(leftSpeed);
            rfDrive.setPower(rightSpeed);
            rbDrive.setPower(rightSpeed);
            Robot.timer.reset();
            boolean turn = true;
            double error = (Robot.gyro.getHeading() - desiredHeading);
            while (Robot.opMode.opModeIsActive() && Math.abs(error) > 0.3  && (Robot.timer.time() < timeoutS)) {

                Robot.gyro.printTelemetry();
                Robot.telemetry.update();
                f.execute();
            }
            f.stop();
            stop();
            resetEncoders();
            System.out.println("\n\n\n");
        }
    }




    /**
     * <u>Auto Method</u>
     * Turns Robot to a given heading using PID control
     * @param inputSpeed - the speed to turn
     * @param desiredHeading - the target heading
     * @param timeoutS - the max amount of time to run method
     */
    public void turnOnHeading(double inputSpeed, double desiredHeading, double timeoutS)
    {
        turnOnHeading(inputSpeed, desiredHeading, timeoutS, new MultiTask() {
            @Override
            public void execute() {

            }
            @Override
            public void stop() {

            }
        });
    }


    /**
     * <u>Auto Method</u>
     * Turns Robot to a given heading using PID control
     * @param inputSpeed - the speed to turn
     * @param desiredHeading - the target heading
     * @param timeoutS - the max amount of time to run method
     * @param f - another robot functionality to run while this loop runs
     */
    public void turnOnHeading(double inputSpeed, double desiredHeading, double timeoutS, MultiTask f)
    {
        if(Robot.opMode.opModeIsActive()) {

            double negation = 1; //turn right
            if(desiredHeading - Robot.gyro.getHeading() > 0) // Check this for correctness
                negation = -1; // turn left

            Robot.timer.reset();


            double p = Constants.TURN_P, i = Constants.TURN_I, d= Constants.TURN_D;
            double minSpeed = 0.3,
                    error = negation * (Robot.gyro.getHeading() - desiredHeading),
                    prevError = error,
                    sumError = 0;
            while (Robot.opMode.opModeIsActive() &&  Math.abs(error) > 0.3 && (Robot.timer.time() < timeoutS)) {
                double currentAngle = Robot.gyro.getHeading();
                error = negation * (currentAngle - desiredHeading);

                double newSpeed = p * error + i * sumError + d * (error - prevError);
                newSpeed = Range.clip(newSpeed, -1, 1 ) ;
                prevError = error;
                if(Math.abs(sumError + error) * i < 1)
                    sumError += error;

                double finalSpeed = negation * (newSpeed) ;

                move(finalSpeed, -finalSpeed, false);
                f.execute();
                Robot.telemetry.addData("Turn P", p);
                Robot.telemetry.addData("Turn I", i);
                Robot.telemetry.addData("Turn D", d);
                Robot.telemetry.addLine("\n");
                Robot.telemetry.addData("Error", error);
                Robot.telemetry.update();
            }
            f.stop();
            stop();
        }
    }




    /**
     * Prints the encoder values of the DcMotors
     * <u>Used for Testing</u>
     */
    public void printEncoders() {
        for (int i = 0; i < motors.length; i++) {
            if (!(motors[i] instanceof DcMotor))
                continue;
            DcMotor m = (DcMotor) motors[i];
            Robot.telemetry.addData("Encoder " + i, m.getCurrentPosition());
        }
    }



    public void printEncodersInInches()
        {
            for (int i = 0; i < motors.length; i++) {
                if (!(motors[i] instanceof DcMotor))
                    continue;
                DcMotor m = (DcMotor) motors[i];
                Robot.telemetry.addData("Inches Encoder " + i, m.getCurrentPosition() / Constants.COUNTS_PER_INCH);
            }
        }


    /**
     * Resets all encoders
     */
    public void resetEncoders() {

        for (int i = 0; i < motors.length; i++) {
            if (!(motors[i] instanceof DcMotor))
                continue;
            DcMotor m = (DcMotor) motors[i];
            m.setPower(0);
            DcMotor.RunMode mode = m.getMode();
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(mode);
        }

    }






    //This stops the robot's drivetrain in teleOp or Auto.
    @Override
    public void stop() {
        move(0,0);
    }
}




