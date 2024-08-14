package org.firstinspires.ftc.teamcode.purePursuit;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.purePursuit.utils.CurvePoint;
import org.firstinspires.ftc.teamcode.purePursuit.utils.RobotMovement;
import org.firstinspires.ftc.teamcode.purePursuit.utils.RobotGlobalVars;
import org.firstinspires.ftc.teamcode.purePursuit.utils.SpeedOmeter;

import java.util.ArrayList;
import java.util.Objects;

@Autonomous(group = "B")
public class StateMachineAuto extends LinearOpMode{
    enum State {
        PATH1,
        PATH2
    }

    private boolean isPaused = false;

    State currentState = State.PATH1;
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RobotGlobalVars.ResetValues();
        RobotMovement.initCurve();

        //PATH 1//
        ArrayList<CurvePoint> path = new ArrayList<>();
        path.add(new CurvePoint(0, 0, 1, 0.5, 30, 30, Math.toRadians(50), 0.5, true, Math.toRadians(-90)));
        path.add(new CurvePoint(-6.97, 113.36, 1, 0.5, 30, 30, Math.toRadians(50), 0.5, true, Math.toRadians(-90)));
        path.add(new CurvePoint(135.85, 75.87, 1, 1.0, 30, 30, Math.toRadians(50), 0.5, false, Math.toRadians(-45)));

        //PATH 2//
        ArrayList<CurvePoint> path2 = new ArrayList<>();
        path2.add(new CurvePoint(126.24, 68.30, 0.7, 1.0, 30, 20, Math.toRadians(50), 0.5, false, Math.toRadians(0)));
        path2.add(new CurvePoint(-5.69, 112.15, 0.7, 1.0, 30, 20, Math.toRadians(50), 0.5, false, Math.toRadians(0)));
        path2.add(new CurvePoint(-54,24,0.7,1.0,30,30,Math.toRadians(50),0.5,false,Math.toRadians(0)));


        waitForStart();


        while(!isStopRequested())
        {
                switch (currentState) {
                    case PATH1:
                    RobotMovement.followCurve(path, Math.toRadians(0));
                    if (RobotMovement.followCurve(path, Math.toRadians(0))) {
                            RobotMovement.initCurve();
                            currentState = State.PATH2;
                    }


                    break;
                    case PATH2:
                        RobotMovement.followCurve(path2, Math.toRadians(0));
                        break;

                }

                drive.setWeightedDrivePower(RobotGlobalVars.ReadPowerValuesRRTranslated());

            RobotMovement.updateAlgorithm(drive);


            Pose2d poseEstimate = drive.getPoseEstimate();
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("follow_x", RobotGlobalVars.ReadFollowPoint().getX());
            telemetry.addData("follow_y", RobotGlobalVars.ReadFollowPoint().getY());
            telemetry.addData("xTranslated", -poseEstimate.getY());
            telemetry.addData("yTranslated", poseEstimate.getX());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("AccHeading", Math.toDegrees(RobotGlobalVars.ReadPosValuesAccumulatedAngle().getHeading()));
            telemetry.addData("Xspeed", SpeedOmeter.getSpeedX());
            telemetry.addData("Yspeed", SpeedOmeter.getSpeedY());
            telemetry.addData("Turnspeed", SpeedOmeter.getRadPerSecond());
            telemetry.update();

        }


    }
}
