package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard1;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp
@Disabled

/*
 THIS IS FOR TESTING!
 I will change the values later!
 NOTE: I am also adding other parts not for the shooter tests in here, such as the vision processer
*/

// TODO: Should we just delete this file? I added '@Disabled' so it doesn't accidentally get used

public class Shooter_test extends OpMode {
    private final int CAMERA_WIDTH = 640;
    ProgrammingBoard1 board = new ProgrammingBoard1();
    boolean yPressed, bPressed, shooter_on, aPressed, dpadDownPressed, dpadUpPressed, xPressed, instructions, rightTriggerPressed, leftTriggerPressed;
    double[] encoderPositions = {8965098675.0, 0.0, 0.3333, 0.6667}; // this is to make our desired positions at spots 1, 2, and 3.
    double leftTargetRPM = 3000, rightTargetRPM = 3000;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    int turntablePosition = 1;
    double closeEnough = 0.01;
    double servoKP = 10;

    @Override
    public void init() {
        board.init(hardwareMap);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, aprilTagProcessor);

    }

    @Override
    public void init_loop() {
        runVisionCode();
    }

    @Override
    public void start() {
        resetRuntime();
    }

    private void runVisionCode() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();
        for (AprilTagDetection detection : currentDetections) {
            idsFound.append(detection.id);
            telemetry.addData("X position", detection.center.x - CAMERA_WIDTH / 2);
            idsFound.append(' ');
        }
        telemetry.addData("April Tags", idsFound);
    }

    @Override

    public void loop() {
        runVisionCode();
        if (gamepad1.right_trigger > 0.7 && !rightTriggerPressed) {
            rightTriggerPressed = true;

            turntablePosition++;
            if (turntablePosition > 3) {
                turntablePosition = 1;
            }
        }
        if (gamepad1.left_trigger < 0.3) {
            leftTriggerPressed = false;
        }
        if (gamepad1.left_trigger > 0.7 && !leftTriggerPressed) {
            leftTriggerPressed = true;
            turntablePosition--;
            if (turntablePosition < 1) {
                turntablePosition = 3;
            }
        }
        if (gamepad1.right_trigger < 0.3) {
            rightTriggerPressed = false;
        }

        if (gamepad1.y && !yPressed) {
            leftTargetRPM += 100;
        }

        if (gamepad1.a && !aPressed) {
            leftTargetRPM -= 100;
        }

        if (gamepad1.dpad_up && !dpadUpPressed) {
            rightTargetRPM += 100;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            rightTargetRPM -= 100;
        }

        leftTargetRPM = Math.max(0, leftTargetRPM);
        rightTargetRPM = Math.max(0, rightTargetRPM);

        if (gamepad1.b && !bPressed) {
            shooter_on = !shooter_on;
        }

        if (shooter_on) {
            board.setShooterLeftSpeed(leftTargetRPM);
            board.setShooterRightSpeed(rightTargetRPM);

        } else {
            board.setShooterLeftSpeed(0);
            board.setShooterRightSpeed(0);
        }


        double servoPower = board.getServoError(encoderPositions[turntablePosition]) * servoKP;
        telemetry.addData("servo power", servoPower);
        board.setServoPower(servoPower);


        telemetry.addData("shooterLeft Speed", board.getShooterLeftSpeed());
        telemetry.addData("shooterRight Speed", board.getShooterRightSpeed());
        telemetry.addData("Shooter average speed", (board.getShooterRightSpeed() + board.getShooterLeftSpeed()) / 2);
        telemetry.addData("Shooter On", shooter_on);
        telemetry.addData("left targetRPM", leftTargetRPM);
        telemetry.addData("right targetRPM", rightTargetRPM);
        telemetry.addData("Encoder position", board.getTurntablePosition());
        telemetry.addData("Turntable Position", turntablePosition);
        telemetry.addLine("press X (Square for playstation controller) for instructions");

        if (gamepad1.x && !xPressed) {
            instructions = !instructions;
        }

        if (instructions) {
            telemetry.addLine("To turn on/off, press B (O for playstation controller). "
                    + "The targetRPM will start at 3000. To adjust the speed, press up and down on"
                    + " the D-pad. It will adjust in 100 unit measures. It is OK that the speed of"
                    + " the motors is fluctuating. This is just a test. The max speed of the motors"
                    + " is ~6000. If you have questions(or need help), please find Orion Howard.");
        }

        dpadUpPressed = gamepad1.dpad_up;
        dpadDownPressed = gamepad1.dpad_down;
        bPressed = gamepad1.b;
        xPressed = gamepad1.x;
        aPressed = gamepad1.a;
        yPressed = gamepad1.y;
        telemetry.update();
    }

    @Override

    public void stop() {
        visionPortal.stopStreaming();
    }


}
