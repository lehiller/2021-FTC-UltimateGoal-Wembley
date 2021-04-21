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

            LeftArm.setDirection(DcMotorSimple.Direction.REVERSE);
            RightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void armup() {
            move(1.0);
        }

        public void armdown() {
            move(-1.0);
        }

        private void move(double speed) {
            LeftArm.setPower(speed);
            RightArm.setPower(speed);
        }

        @Override
        public void run() {
            if (Robot.g2.dpad_up) {
                LeftArm.setTargetPosition(135);
                RightArm.setTargetPosition(135);
                LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftArm.setVelocity(400);
                RightArm.setVelocity(400);
            } else if (Robot.g2.dpad_left) {
                LeftArm.setTargetPosition(50);
                RightArm.setTargetPosition(50);
                LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftArm.setVelocity(1200);
                RightArm.setVelocity(1200);

            } else {
                LeftArm.setTargetPosition(0);
                RightArm.setTargetPosition(0);
                LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftArm.setVelocity(1200);
                RightArm.setVelocity(1200);
            }
        }
            @Override
            public void stop() { move(0); }



    }

