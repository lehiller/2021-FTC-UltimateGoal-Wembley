package org.firstinspires.ftc.teamcode.sensors;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Robot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Locale;



public class Gyro {

    private BNO055IMU imu;

    private double  globalAngle, power = .30, correction;
    private Orientation lastAngles = new Orientation();

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double getCorrection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public double getHeading()
    {
        getAngle();
        return lastAngles.firstAngle;
    }

    /**
     * Outputs information to telemetry
     */
    public void print()
    {
        //   getAngle();
        Robot.telemetry.addData("1 imu heading", lastAngles.firstAngle);
        Robot.telemetry.addData("2 global heading", globalAngle);
        Robot.telemetry.addData("3 correction", correction);
        Robot.telemetry.addData("calib", imu.getCalibrationStatus().toString());
    }

    public void testPrint()
    {

        getAngle();
        System.out.println("IMU Heading: " + lastAngles.firstAngle);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

}
