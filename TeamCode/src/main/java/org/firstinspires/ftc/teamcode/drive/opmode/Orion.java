package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard1;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp

public class Orion extends OpMode {
    ProgrammingBoard1 board = new ProgrammingBoard1();


    @Override
    public void init() {
        board.init(hardwareMap);

    }



    @Override
    public void init_loop() {
        //filler
    }

    @Override
    public void start() {
        resetRuntime();
    }

    boolean shooter_on;
    boolean aPressed;
    boolean dpadDownPressed;
    boolean dpadUpPressed;
    double targetRPM = 3000;

    @Override
    public void loop() {
        if (targetRPM <= 0){
            targetRPM = 0;
        }
        if (gamepad1.dpad_up && !dpadUpPressed) {
            targetRPM += 100;
        }
        if (gamepad1.dpad_down && !dpadDownPressed) {
            targetRPM -= 100;
        }
        if (gamepad1.a && !aPressed) {
            shooter_on = !shooter_on;
        }
        if (shooter_on) {
            board.setShooterLeftSpeed(targetRPM);
            board.setShooterRightSpeed(targetRPM);

        } else {
            board.setShooterLeftSpeed(0);
            board.setShooterRightSpeed(0);
        }
        telemetry.addData("shooterLeft Speed", board.getShooterLeftSpeed());
        telemetry.addData("shooterRight Speed", board.getShooterRightSpeed());
        telemetry.addData("Shooter On", shooter_on);
        telemetry.addData("targetRPM", targetRPM);
        dpadUpPressed = gamepad1.dpad_up;
        dpadDownPressed = gamepad1.dpad_down;
        aPressed = gamepad1.a;
    }
}