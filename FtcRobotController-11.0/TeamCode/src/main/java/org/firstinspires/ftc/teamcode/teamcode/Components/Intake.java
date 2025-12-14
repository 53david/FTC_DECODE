package org.firstinspires.ftc.teamcode.teamcode.Components;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.teamcode.OpModes.Teleop.dashboard;
import static org.firstinspires.ftc.teamcode.teamcode.OpModes.Teleop.gm1;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.sql.Time;

public class Intake {
    Telemetry telemetry;
    private DcMotorEx intakeMotor;
    private CRServo servo;
    public Intake(DcMotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void update() {
        boolean x = gm1.left_bumper;
        boolean y = gm1.right_bumper;
        if (x==true)
        intakeMotor.setPower(1);
        else if (y==true) {
            intakeMotor.setPower(-1);
        }

    }
    public void run(){
        intakeMotor.setPower(1);
    }

}