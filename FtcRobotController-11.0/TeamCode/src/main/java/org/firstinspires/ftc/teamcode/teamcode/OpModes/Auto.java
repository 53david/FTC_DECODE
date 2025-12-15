package org.firstinspires.ftc.teamcode.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.Components.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.teamcode.Components.Outake;
import org.firstinspires.ftc.teamcode.teamcode.Components.Storage;
import org.firstinspires.ftc.teamcode.teamcode.Components.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="Mascul fioros")
public class Auto extends LinearOpMode {
    private DriveTrain chassis; private Intake intake; private CRServo servo; public static Vision vision;
    private Outake outake; Servo transfer; private Storage storage;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    public static PIDCoefficients coefs = new PIDCoefficients(0.25 ,0.003, 0.004);
    DcMotorEx intakeMotor,rotate,leftFront,leftBack,rightBack,rightFront,shoot1,shoot2;
    ElapsedTime timer = new ElapsedTime();
    WebcamName webcam1;
    RevColorSensorV3 colorSensor1,colorSensor2;
    public boolean ok1=true,ok2=true,ok3=true;
    public static Telemetry dashboard;
    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        waitForStart();
        while(opModeIsActive()){
            if (timer.seconds()<30) {
                if (timer.seconds()<1.75) {
                    chassis.backward();
                }
                if (timer.seconds()>1.75 && timer.seconds()<3){
                    chassis.stop();
                }
                intake.run();
                storage.update();
                storage.Index();
                outake.shoot();
                outake.run();
                if (timer.seconds() > 15 && timer.seconds()<16.75) {
                    chassis.strafe();
                }
                if (timer.seconds()>16.75 && timer.seconds()<17.50){
                    chassis.stop();
                    terminateOpModeNow();
                }
            }
        }
    }
    private void initializeHardware() {
        servo = hardwareMap.get(CRServo.class,"servo");
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        MotorConfigurationType m= leftFront.getMotorType();
        m.setAchieveableMaxRPMFraction(1);
        leftFront.setMotorType(m);
        rightFront.setMotorType(m);
        leftBack.setMotorType(m);
        rightFront.setMotorType(m);
        dashboard = FtcDashboard.getInstance().getTelemetry();
        transfer = hardwareMap.get(Servo.class,"transfer");
        shoot1 = hardwareMap.get(DcMotorEx.class,"shoot1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"shoot2");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate");
        webcam1 = hardwareMap.get(WebcamName.class,"webcam1");
        chassis = new DriveTrain(leftFront,rightFront,leftBack,rightBack);
        intake = new Intake(intakeMotor);
        outake = new Outake(shoot1,shoot2,rotate,transfer,telemetry);
        storage = new Storage(servo,shoot1,intakeMotor);
        vision = new Vision(rotate,webcam1);
        storage.turner.setPidCoefficients(coefs);

    }

}
