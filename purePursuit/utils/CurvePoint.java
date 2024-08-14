package org.firstinspires.ftc.teamcode.purePursuit.utils;

import org.opencv.core.Point;
/**
 * CurvePoint is a class used with the followCurve function in MovementEssentials.
 */
public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;
    public double pointLength;
    public boolean globalAngleLock;
    public double individualFollowAngle;


    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed,
                      double followDistance, double slowDownTurnRadians, double slowDownTurnAmount){
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.pointLength = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }


    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed,
                      double followDistance,double pointLength, double slowDownTurnRadians, double slowDownTurnAmount, boolean globalAngleLock, double individualFollowAngle){
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.pointLength = pointLength;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
        this.globalAngleLock = globalAngleLock;
        this.individualFollowAngle = individualFollowAngle;
    }

    public CurvePoint(CurvePoint nextPoint) {
        x = nextPoint.x;
        y = nextPoint.y;
        moveSpeed = nextPoint.moveSpeed;
        turnSpeed = nextPoint.turnSpeed;
        followDistance = nextPoint.followDistance;
        slowDownTurnRadians = nextPoint.slowDownTurnRadians;
        slowDownTurnAmount = nextPoint.slowDownTurnAmount;
        pointLength = nextPoint.pointLength;
        globalAngleLock = nextPoint.globalAngleLock;
        individualFollowAngle = nextPoint.individualFollowAngle;

    }

    public Point toPoint(){
        return new Point(x,y);
    }
    public void setPoint(Point p){
        x = p.x;
        y = p.y;
    }
}