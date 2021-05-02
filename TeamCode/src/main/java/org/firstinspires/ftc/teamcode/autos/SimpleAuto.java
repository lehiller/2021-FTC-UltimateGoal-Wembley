package org.firstinspires.ftc.teamcode.autos;



//drive forward to line, shoot 3 rings, park on line.


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Robot;


public class SimpleAuto extends LinearOpMode {


   private Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2, this);

        // anything up here will happen after init but before play  (cameras, zeroing...)

        waitForStart();
        //anything here  happens once after play.


        //just drive

        robot.driveTrain.encoderDrive( .8, 12, 12,5);

        //drive and get ready to shoot
        MultiTask spinUp = new MultiTask() {
            @Override
            public void execute() {
                robot.ringShooter.spinUp();
            }

            @Override
            public void stop() {

            }
        };

        robot.driveTrain.encoderDrive( .8, 12, 12,5, spinUp);



    }
}
