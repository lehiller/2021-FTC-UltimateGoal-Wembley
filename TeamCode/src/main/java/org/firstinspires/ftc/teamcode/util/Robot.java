package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.sensors.Color;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.RingGrabber;
import org.firstinspires.ftc.teamcode.subsystems.RingShooter;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGrabber;


public class Robot  extends RobotBase {


    public DriveTrain driveTrain;
    public Arm arm;
    public RingGrabber ringGrabber;
    public RingShooter ringShooter;
    public WobbleGrabber wobbleGrabber;
    public static Camera camera;
    public static Gyro gyro;
    public static org.firstinspires.ftc.teamcode.sensors.Color color;




    public Robot(HardwareMap hardwareMap, Telemetry t, Gamepad gamepad1, Gamepad gamepad2) {
        super(hardwareMap, t, gamepad1, gamepad2);
    }

    public Robot(HardwareMap hardwareMap, Telemetry t, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode opMode) {
        super(hardwareMap, t, gamepad1, gamepad2, opMode);
    }

    @Override
    public void initSubsystems() {
        arm = new Arm();
        driveTrain = new DriveTrain();
        ringGrabber = new RingGrabber();
        ringShooter = new RingShooter();
        wobbleGrabber = new WobbleGrabber();
        camera = new Camera();
        gyro = new Gyro();
        color = new Color();


    }
}
