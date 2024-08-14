package org.firstinspires.ftc.teamcode.purePursuit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(group = "apathplanning")
public class PathPlanningTeleOp extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RobotGlobalVars.ResetValues();
        //drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));

        ElapsedTime waitTimer = new ElapsedTime();
        telemetry.addLine("TeleOp program for planning out paths");
        telemetry.addLine(" ");
        telemetry.addLine("First two values are roadrunner coordinates");
        telemetry.addLine(" ");
        telemetry.addLine("Second two values are pure pursuit coordinates (90 deg rotation from roadrunner)");
        telemetry.addLine(" ");
        telemetry.addLine("heading AKA rotation values are included at the end");
        telemetry.addLine(" ");
        telemetry.addLine("Press A to set coordinates to [robot starting position] **Default is 0,0");
        telemetry.addLine(" ");
        telemetry.addLine("[NOTE]: Roadrunner uses inches and pure pursuit uses centimeters");

        telemetry.update();
        waitForStart();
        waitTimer.reset();
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            if (gamepad1.a) {
                //replace pose values with robot starting position relative to the bottom left corner of the field
                drive.setPoseEstimate(new Pose2d(0,0,0));
            }

            //update drivetrain based on input powers
            drive.update();
            //send robot current position and rotation to RobotGlobalVars
            RobotGlobalVars.WritePosValues_Pose(drive.getPoseEstimate());


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Roadrunner X (cm)", MathFunctions.toCentimeters(poseEstimate.getX()));
            telemetry.addData("Roadrunner Y (cm)", MathFunctions.toCentimeters(poseEstimate.getY()));
            telemetry.addData("Pure Pursuit X (cm)", MathFunctions.toCentimeters(-poseEstimate.getY()));
            telemetry.addData("Pure Pursuit Y (cm)", MathFunctions.toCentimeters(poseEstimate.getX()));
            telemetry.addLine(" ");
            telemetry.addData("Roadrunner X (in)", poseEstimate.getX());
            telemetry.addData("Roadrunner Y (in)", poseEstimate.getY());
            telemetry.addData("Pure Pursuit X (in)", -poseEstimate.getY());
            telemetry.addData("Pure Pursuit Y (in)", poseEstimate.getX());
            telemetry.addLine(" ");
            telemetry.addData("current heading", poseEstimate.getHeading());
            telemetry.addData("Accumulated heading", Math.toDegrees(RobotGlobalVars.ReadPosValuesAccumulatedAngle().getHeading()));
            telemetry.update();

        }
    }
}
