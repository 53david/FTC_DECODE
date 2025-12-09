package org.firstinspires.ftc.teamcode.teamcode.Components;

import static org.firstinspires.ftc.teamcode.teamcode.OpModes.Teleop.dashboard;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Storage {
    private CRServo revolver;
    double p = 1.0, t=200; boolean ok;
    ElapsedTime timer = new ElapsedTime();
    int n=0;
    public Storage (CRServo revolver){
        this.revolver=revolver;
    }
    public void update (Gamepad gamepad1) {
        if (gamepad1.triangle && ok==true){
            ok=false; return;
        }
        else if (gamepad1.triangle && ok==false){
            ok=true; return;
        }
        if (ok == true) {
            if (timer.milliseconds() < t) {
                revolver.setPower(p);
            } else if (timer.milliseconds() < t + 25) {
                revolver.setPower(-0.5);
            } else if (timer.milliseconds() < 1100) {
                revolver.setPower(0);
            } else {
                n++;
                timer.reset();
            }
            dashboard.addData("time", timer.milliseconds());
        }
        else {
            timer.reset(); return;
        }

    }

}
