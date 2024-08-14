package org.firstinspires.ftc.teamcode.purePursuit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.purePursuit.utils.CurvePoint;
import org.firstinspires.ftc.teamcode.purePursuit.utils.MathFunctions;
import org.firstinspires.ftc.teamcode.purePursuit.utils.RobotMovement;
import org.firstinspires.ftc.teamcode.purePursuit.utils.RobotGlobalVars;
import org.firstinspires.ftc.teamcode.purePursuit.utils.SpeedOmeter;

import java.util.ArrayList;
import java.util.Objects;

@Autonomous(group = "B")
public class StraightedgeAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RobotGlobalVars.ResetValues();
        RobotMovement.initCurve();
        RobotMovement.resetPathTimers();
        //drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));

        waitForStart();
        RobotMovement.resetPathTimers();
        while (!isStopRequested()) {
            //RobotMovement.gunToPosition(76.2, 0, Math.toRadians(0), 0.5, 0.5,Math.toRadians(40),Math.toRadians(0),true);
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.3, 1.0, 30, 30, Math.toRadians(50), 0.0,false,Math.toRadians(0)));
            allPoints.add(new CurvePoint(5, 91.5, 0.4, 1, 30, 30, Math.toRadians(50), 0.0,false,Math.toRadians(0)));
            allPoints.add(new CurvePoint(92, 91.5, 0.3, 1, 30, 30, Math.toRadians(50), 0.0,true,Math.toRadians(0)));
            //allPoints.add(new CurvePoint(130, 20, 1.0, 1, 30, 30, Math.toRadians(50), 0.0,true,Math.toRadians(90)));
            RobotMovement.followCurve(allPoints,Math.toRadians(0));
            telemetry.addData("Path Segment Percent: ", MathFunctions.progressBarString(RobotMovement.getPathSegmentPercentCompleted(allPoints, RobotMovement.getCurrentPathSegmentIndex(allPoints)-1,RobotMovement.getCurrentPathSegmentIndex(allPoints))));
            telemetry.addData("Path Percent: ", MathFunctions.progressBarString(RobotMovement.getPathPercentCompleted(allPoints)));
            telemetry.addData("Path Timer: ", RobotMovement.getPathTimerSeconds());
            telemetry.addData("Path Segment Timer: ", RobotMovement.getPathSegmentTimerSeconds(allPoints));

                //set calculated power to drivetrain
                drive.setWeightedDrivePower(RobotGlobalVars.ReadPowerValuesRRTranslated());
            //send and pull important data to/from the math classes
            //also updates drivetrain powers with the latest inputs
            RobotMovement.updateAlgorithm(drive);


            Pose2d poseEstimate = drive.getPoseEstimate();
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            //telemetry.addData("follow_x", RobotGlobalVars.ReadFollowPoint().getX());
            //telemetry.addData("follow_y", RobotGlobalVars.ReadFollowPoint().getY());
            telemetry.addData("xTranslated", -poseEstimate.getY());
            telemetry.addData("yTranslated", poseEstimate.getX());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("AccHeading", Math.toDegrees(RobotGlobalVars.ReadPosValuesAccumulatedAngle().getHeading()));
            //telemetry.addData("Xspeed", SpeedOmeter.getSpeedX());
            //telemetry.addData("Yspeed", SpeedOmeter.getSpeedY());
            //telemetry.addData("Turnspeed", SpeedOmeter.getRadPerSecond());
            telemetry.update();

        }
    }
}

