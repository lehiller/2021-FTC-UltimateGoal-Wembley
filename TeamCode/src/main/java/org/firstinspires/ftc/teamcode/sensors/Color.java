package org.firstinspires.ftc.teamcode.sensors;



import org.firstinspires.ftc.teamcode.util.RobotMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import static org.firstinspires.ftc.teamcode.util.RobotMap.LeftColor;
import static org.firstinspires.ftc.teamcode.util.RobotMap.RightColor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Color {


    RobotMap.LeftColor.enableLed(true);
    RobotMap.RightColor.enableLed(true);



    public double seeBlue()
    {
    return LeftColor.blue();
    telemetry.addData("blue = " , LeftColor.blue());
    }

    public double seeRed()
    {
     return LeftColor.red();
    }

    public double seeWhite()
    {
    return LeftColor.alpha();
    }





}



