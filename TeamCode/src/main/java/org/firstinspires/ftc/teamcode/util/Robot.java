package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;



public class Robot  extends RobotBase {


    public DriveTrain DriveTrain;
    public Arm Arm;
    public RingGrabber RingGrabber;
    public RingShooter RingShooter;
    public WobbleGrabber WobbleGrabber;
    public static Camera Camera;
    public static Gyro Gyro;
    public static org.firstinspires.ftc.teamcode.sensors.Color Color;




    public Robot(HardwareMap hardwareMap, Telemetry t, Gamepad gamepad1, Gamepad gamepad2) {
        super(hardwareMap, t, gamepad1, gamepad2);
    }

    public Robot(HardwareMap hardwareMap, Telemetry t, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode opMode) {
        super(hardwareMap, t, gamepad1, gamepad2, opMode);
    }

    @Override
    public void initSubsystems() {
        Arm = new Arm();
        DriveTrain = new DriveTrain();
        RingGrabber = new RingGrabber();
        RingShooter = new RingShooter();
        WobbleGrabber = new WobbleGrabber();
        Camera = new Camera();
        Gyro = new Gyro();
        Color = new Color();


    }
}
