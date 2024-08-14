package org.firstinspires.ftc.teamcode.purePursuit;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.purePursuit.utils.CurvePoint;
import org.firstinspires.ftc.teamcode.purePursuit.utils.MathFunctions;
import org.firstinspires.ftc.teamcode.purePursuit.utils.RobotGlobalVars;
import org.firstinspires.ftc.teamcode.purePursuit.utils.RobotMovement;

import java.util.ArrayList;

@Autonomous(group = "B", name = "PathTriggerAuto")
public class PathTriggerAuto extends LinearOpMode {

    enum State {
        PATH1,

        WAIT,

        PATH2,

        HOLD_POS

    }

    State currentState = State.PATH1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Call this only once during init
        RobotGlobalVars.ResetValues();
        //Call this each time before running a new path
        RobotMovement.initCurve();
        //Call this before each new path to reset the timers
        RobotMovement.resetPathTimers();

        //PATH1//
        ArrayList < CurvePoint > path1 = new ArrayList < > ();
        path1.add(new CurvePoint(0, 0, 1, 1, 30, 30, Math.toRadians(50), 0.0, false, Math.toRadians(0)));
        path1.add(new CurvePoint(0, 113, 1, 1, 30, 30, Math.toRadians(50), 0.0, false, Math.toRadians(0)));

        //PATH2//
        ArrayList < CurvePoint > path2 = new ArrayList < > ();
        path2.add(new CurvePoint(0, 113, 1, 1, 30, 30, Math.toRadians(50), 0.0, false, Math.toRadians(0)));
        path2.add(new CurvePoint(130, 113, 1, 1, 30, 30, Math.toRadians(50), 0.0, false, Math.toRadians(0)));

        waitForStart();
        RobotMovement.resetPathTimers();
        while (!isStopRequested()) {
            switch (currentState) {
                case PATH1:
                    RobotMovement.followCurve(path1, Math.toRadians(0));
                    telemetry.addData("Path Segment Percent: ", MathFunctions.progressBarString(RobotMovement.getPathSegmentPercentCompleted(path1, RobotMovement.getCurrentPathSegmentIndex(path1) - 1, RobotMovement.getCurrentPathSegmentIndex(path1))));
                    if (RobotMovement.followCurve(path1, Math.toRadians(0))) {
                        RobotMovement.resetPathTimers();
                        RobotMovement.initCurve();
                        currentState = State.WAIT;
                    }

                    break;

                case WAIT:
                    RobotMovement.followCurve(path1, Math.toRadians(0));
                    if (RobotMovement.getPathTimerSeconds() > 3) {
                        RobotMovement.initCurve();
                        currentState = State.PATH2;
                    }

                    break;

                case PATH2:
                    RobotMovement.followCurve(path2, Math.toRadians(0));

                    if (RobotMovement.getPathPercentCompleted(path2) > 50) {
                        //servo.setPosition(1);
                        RobotMovement.initCurve();
                        RobotMovement.resetPathTimers();
                        currentState = State.HOLD_POS;
                    }

                    break;
                case HOLD_POS:
                    RobotMovement.followCurve(path2, Math.toRadians(0));
                    break;



            }
            telemetry.update();
            drive.setWeightedDrivePower(RobotGlobalVars.ReadPowerValuesRRTranslated());
            RobotMovement.updateAlgorithm(drive);
        }


    }


}