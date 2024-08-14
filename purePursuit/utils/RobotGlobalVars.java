package org.firstinspires.ftc.teamcode.purePursuit.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

public class RobotGlobalVars {
    public static double current_x;
    public static double current_y;
    public static double current_heading;

    public static double target_x_power;
    public static double target_y_power;
    public static double target_heading_power;

    public static double follow_x;
    public static double follow_y;

    public static double x_speed;
    public static double y_speed;
    public static double heading_speed;

    static double headingAccumulator = 0;
    static double lastHeading = 0;

    public static void WritePosValues_Individual(double currentx, double currenty, double currentheading) {
        current_x = currentx;
        current_y = currenty;
        current_heading = currentheading;
    }
    public static void WritePosValues_Pose(Pose2d values) {
        current_x = values.getX();
        current_y = values.getY();
        current_heading = values.getHeading();
    }

    public static void WritePowerValues_Individual(double powx, double powy, double powheading) {
        target_x_power = powx;
        target_y_power = powy;
        target_heading_power = powheading;
    }
    public static void WritePowerValues_Pose(Pose2d values) {
        target_x_power = values.getX();
        target_y_power = values.getY();
        target_heading_power = values.getHeading();
    }
    public static void WriteFollowPoint(CurvePoint point) {
        follow_x = point.x;
        follow_y = point.y;
    }
    public static void WriteSpeedValues(Pose2d values) {
        x_speed = values.getX();
        y_speed = values.getY();
        heading_speed = values.getHeading();
    }
    public static void WritePowerXValue(double x_value) {
        target_x_power = x_value;
    }
    public static void WritePowerYValue(double y_value) {
        target_y_power = y_value;
    }
    public static void WritePowerHeadingValue(double heading_value) {
        target_heading_power = heading_value;
    }

    public static Pose2d ReadPosValues() {
        return new Pose2d(-current_y*2.54,current_x*2.54,current_heading);
    }
    public static Pose2d ReadSpeedValues() {
        return new Pose2d(-y_speed*2.54,x_speed*2.54,heading_speed);
    }
    public static Vector2d ReadFollowPoint() {
        return new Vector2d(follow_x,follow_y);
    }
    public static Pose2d ReadPosValuesAccumulatedAngle() {

        double heading = current_heading;
        double deltaHeading = heading - lastHeading;

        headingAccumulator += Angle.normDelta(deltaHeading);
        lastHeading = heading;

        return new Pose2d(-current_y*2.54,current_x*2.54,headingAccumulator);
    }

    public static Pose2d ReadPowerValues() {
        return new Pose2d(target_x_power,target_y_power,target_heading_power);
    }
    public static Pose2d ReadPowerValuesRRTranslated() {
        return new Pose2d(-target_x_power,-target_y_power,target_heading_power);
    }


    public static void ResetValues() {
           current_x = 0;
           current_y = 0;
           current_heading = 0;

           target_x_power = 0;
           target_y_power = 0;
           target_heading_power = 0;

           x_speed = 0;
           y_speed = 0;
           heading_speed = 0;

          headingAccumulator = 0;
          lastHeading = 0;
    }
}
