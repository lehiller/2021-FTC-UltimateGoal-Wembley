package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;
//import org.firstinspires.ftc.teamcode.util.record.Recordable;

    public class Arm extends Subsystem {

        private DcMotorEx LeftArm, RightArm;

        public Arm() {
            super("Arm");
            LeftArm = Robot.hw.get(DcMotorEx.class, RobotMap.LEFT_ARM_ID);
            RightArm = Robot.hw.get(DcMotorEx.class, RobotMap.RIGHT_ARM_ID);

            LeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            holdArm(true);
        }






        @Override
        public void run() {
            if (Robot.g2.dpad_up) {
                armUp();
            }
            else if (Robot.g2.dpad_left) {
                armMid();

            }
            else if (Robot.g2.dpad_down) {
                armDown();

            }
        }

        //here are our Auto Methods for the Arm

            public void resetArms() {

                LeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }


            public void armDown() {
                LeftArm.setTargetPosition(0);
                RightArm.setTargetPosition(0);
                LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftArm.setVelocity(200);
                RightArm.setVelocity(200);
                Robot.telemetry.addData("Arm Position", LeftArm.getCurrentPosition());


            }

            public void armMid() {
                LeftArm.setTargetPosition(50);
                RightArm.setTargetPosition(50);
                LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftArm.setVelocity(1200);
                RightArm.setVelocity(1200);
                Robot.telemetry.addData("Arm Position", LeftArm.getCurrentPosition());

            }

            public void armUp() {
                LeftArm.setTargetPosition(135);
                RightArm.setTargetPosition(135);
                LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);LeftArm.setVelocity(1200);
                RightArm.setVelocity(400);
                Robot.telemetry.addData("Arm Position", LeftArm.getCurrentPosition());

            }


            public void holdArm( boolean turnOn) {
            if (turnOn ) {
                LeftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                LeftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                RightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            Robot.telemetry.addData("Arm Position", LeftArm.getCurrentPosition());

        }


                @Override
                public void stop () {

                }



    }

