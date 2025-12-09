package org.firstinspires.ftc.teamcode.teamcode.Components;


import static org.firstinspires.ftc.teamcode.teamcode.OpModes.Teleop.dashboard;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Outake {

    private DcMotorEx shoot1, shoot2,rotate;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    double fx=752.848, fy=752.848, cx=314.441, cy=219.647;
    double mt=112; int maxP=700;
    double gr=13.7;  int minP=-350; private Servo transfer;
    public Outake(DcMotorEx shoot1,DcMotorEx shoot2,DcMotorEx rotate, Servo transfer) {
        this.shoot1 = shoot1;
        this.shoot2 = shoot2;
        this.rotate = rotate;
        this.transfer=transfer;
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    public void shooter(Gamepad gamepad1) {
        int sp=gamepad1.square ? 1 : 0;
        shoot1.setPower(sp);
        shoot2.setPower(-sp);
        rotate.setPower(gamepad1.right_stick_x * 0.25);
        int y = rotate.getCurrentPosition();
        if (y<minP){
            rotate.setTargetPosition(maxP);
            rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotate.setPower(1);
        }
        else if (y>maxP){
            rotate.setTargetPosition(minP);
            rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotate.setPower(1);
        }
        int x = gamepad1.circle? 1:0;
        if (x==1)
            transfer.setPosition(0);
        else {
            transfer.setPosition(1);
        }
    }


    }
}

