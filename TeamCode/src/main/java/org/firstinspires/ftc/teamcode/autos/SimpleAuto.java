package org.firstinspires.ftc.teamcode.autos;





import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.sensors.Color;
import org.firstinspires.ftc.teamcode.sensors.Gyro;



@Autonomous(name = "SimpleAuto", group = "autos")
public class SimpleAuto extends LinearOpMode {


   private Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2, this);

        // anything up here will happen after init but before play  (cameras, zeroing...)
        robot.initSubsystems();



        robot.gyro.getHeading();
        robot.gyro.printTelemetry();



        robot.camera.detect();

        waitForStart();
        //anything here  happens once after play.


        //just drive

        robot.driveTrain.encoderDrive( .4, 57, 58,5);

        robot.driveTrain.turnOnHeading(.5,90,3);



        /*

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

           */

    }
}
