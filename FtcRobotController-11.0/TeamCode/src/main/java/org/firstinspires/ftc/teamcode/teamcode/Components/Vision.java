package org.firstinspires.ftc.teamcode.teamcode.Components;

import static org.firstinspires.ftc.teamcode.teamcode.OpModes.Teleop.dashboard;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Vision {
    public DcMotorEx rotate;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    double fx=752.848, fy=752.848, cx=314.441, cy=219.647;
    double n =10/3;
    private WebcamName webcam;

    public Vision(DcMotorEx rotate,WebcamName webcam){
        this.rotate=rotate;
        this.webcam=webcam;
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setLensIntrinsics(fx,fy,cx,cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal,10);

    }

    public void update() {
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections.isEmpty()) {
            dashboard.addLine("No tag detected.");
            return;
        } else {
            AprilTagDetection tag = detections.get(0);
            dashboard.addData("deg", tag.robotPose);
            if (tag.ftcPose == null) {
                dashboard.addData("Tag id ", tag.id);
                dashboard.addLine("tag not valid");
                return; 
            }
            if (tag.id == 20) {
                int a = (int) (tag.ftcPose.bearing);
                int target = (int) (a * n);
                    rotate.setTargetPosition(target);
                    rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rotate.setPower(0.25);
                    dashboard.addData("target", a);
                dashboard.addData("deg", tag.ftcPose.bearing);
                dashboard.addData("pos", rotate.getCurrentPosition());

                dashboard.update();
            }
    }

    }




}