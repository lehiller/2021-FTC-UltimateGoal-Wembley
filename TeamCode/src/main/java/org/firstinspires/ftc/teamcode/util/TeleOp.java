package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp("WembleyTeleOp" , group="Iterative Opmode")

public class TeleOp extends OpMode {

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
     * Runs the robot movements
     */
    @Override
    public void loop() {
        robot.run();
    }

    /**
     * Stops all Robot movements
     */
    @Override
    public void stop() {
        robot.stop();
    }




}
