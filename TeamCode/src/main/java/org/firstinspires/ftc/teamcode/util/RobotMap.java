package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotMap {

    //Name the Drive Motors
    public static final String LEFT_FRONT_DRIVE_ID = "LeftFront";
    public static final String  LEFT_BACK_DRIVE_ID = "LeftBack";
    public static final String RIGHT_FRONT_DRIVE_ID = "RightFront";
    public static final String RIGHT_BACK_DRIVE_ID = "RightBack";

    //Name the Arm motors
    public static final String LEFT_ARM_ID = "LeftArm";
    public static final String RIGHT_ARM_ID = "RightArm";


    //Name the shooter motors
    public static final String LEFT_OUT_ID = "LeftOut";
    public static final String RIGHT_OUT_ID = "RightOut";

    //Name the servos
    public static final String FINGER_ID = "RingGrabber";
    public static final String CLAW_ID = "WobbleGrabber";
    public static final String TRIGGER_ID = "Trigger";
    public static final String BUMP_ID = "Bump";

    //Name the sensors that are in Config

    public static final String LEFT_COLOR_SENSOR_ID = "LeftColor";
    public static final String RIGHT_COLOR_SENSOR_ID = "RightColor";
}
