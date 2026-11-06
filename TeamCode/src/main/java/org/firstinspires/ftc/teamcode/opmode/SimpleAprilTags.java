/*package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@Autonomous
public class SimpleAprilTags extends OpMode {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init(){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal =VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);
    }
    @Override
    void init_loop(){

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();

        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleTag = (AprilTagSingleDetection) detection;
                int tagId = singleTag.id;
                idsFound.append(tagId);
                idsFound.append(" ");
                // Handle standalone tag telemetry or movement logic

            } else if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String clusterName = cluster.metadata.name;
                idsFound.append()
                // Handle cluster-based localization (e.g., BIOBUZZ center-cell alignment)
            }
        }

        telemetry.addData("April Tages", idsFound);
    }

    @Override
    public void start(){
        visionPortal.stopStreaming();
    }

    @Override
    public void loop(){

    }
}
*/