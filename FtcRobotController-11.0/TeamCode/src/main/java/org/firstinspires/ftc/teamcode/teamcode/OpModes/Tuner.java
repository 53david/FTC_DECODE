package org.firstinspires.ftc.teamcode.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teamcode.Components.Storage;
import org.firstinspires.ftc.teamcode.teamcode.Stuff.PIDController;

@TeleOp(name = "Tuner")
@Config
public class Tuner extends LinearOpMode {
    Storage storage;
    private CRServo servo; Servo transfer;
    private DcMotorEx shoot1,shoot2;
    public static boolean next = false,prevnext = false;

    public static PIDCoefficients coefs = new PIDCoefficients(0.45,0.0002, 0.0003);
    public static Telemetry dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        dashboard= FtcDashboard.getInstance().getTelemetry();
        storage.turner.setPidCoefficients(coefs);
        waitForStart();
        while (opModeIsActive()){

            dashboard.update();
            storage.update();
            if (next!=prevnext && next) {
                storage.Turn120();
                next = false;
                prevnext = false;
            }

        }

        }
    private void initializeHardware(){
        servo = hardwareMap.get(CRServo.class,"servo");
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2= hardwareMap.get(DcMotorEx.class,"intake");
        storage = new Storage(servo,shoot1,shoot2);
        storage.turner.setPidCoefficients(coefs);

    }

}
