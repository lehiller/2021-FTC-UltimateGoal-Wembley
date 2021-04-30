package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "WembleyTeleOpAS", group="Iterative Opmode")
public class TeleOperated extends OpMode {

    private Robot robot;
    /**
     * Intializes the objects within the Robot class
     */
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.DriveTrain.setBrakeMode(false);
    }

    /**
     * Runs the robot movements when play is pressed on driver phone
     */
    @Override
    public void loop() {
        robot.run();
    }

    /**
     * Stops all Robot movements when stop is pressed on driver phone
     */
    @Override
    public void stop() {
        robot.stop();
    }




}
