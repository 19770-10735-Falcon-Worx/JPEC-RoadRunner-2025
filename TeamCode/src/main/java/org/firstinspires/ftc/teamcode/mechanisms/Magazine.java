package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Magazine {

    private DcMotorEx magEncoder;
    private CRServo magServo;

    public Magazine(HardwareMap hwMap) {
        magEncoder = hwMap.get(DcMotorEx.class, "intake"); // Mag uses intake encoder
        magEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        magServo = hwMap.get(CRServo.class, "turntable");
    }

    public SequentialAction magSpin() {
        return new SequentialAction(

        );
    }

    private double getTurntablePosition() {

        if (magEncoder.getCurrentPosition() < 0) {
            return 1 - Math.abs((magEncoder.getCurrentPosition() / 8192.0) % 1);
        }
        return (magEncoder.getCurrentPosition() / 8192.0) % 1;
    }

    private void setServoPower(double power) {
        magServo.setPower(power);
    }

    private double getServoError(double target) {
        double servoError = target - getTurntablePosition();
        servoError = (servoError + 0.5) % 1;
        if (servoError < 0) {
            servoError += 1;
        }
        servoError -= 0.5;

        return servoError;
        //negative is because the servo isn't inverted
    }
}