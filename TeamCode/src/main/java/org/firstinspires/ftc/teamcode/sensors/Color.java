package org.firstinspires.ftc.teamcode.sensors;



import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.RobotMap;

import com.qualcomm.robotcore.hardware.ColorSensor;


public class Color {

    private ColorSensor leftColor, rightColor;

    public Color ()
    {
        leftColor= Robot.hw.get(ColorSensor.class, RobotMap.LEFT_COLOR_SENSOR_ID);
        rightColor= Robot.hw.get(ColorSensor.class, RobotMap.RIGHT_COLOR_SENSOR_ID);
    }


    public double seeBlue()
    {
        Robot.telemetry.addData("blue = " , leftColor.blue());
        return leftColor.blue();

    }

    public double seeRed()
    {
        Robot.telemetry.addData("red= " , leftColor.red());
        return leftColor.red();
    }

    public double seeWhite()
    {
        Robot.telemetry.addData("white = " , leftColor.alpha());
        return leftColor.alpha();
    }





}



