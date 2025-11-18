package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.time.Clock;
import java.util.Timer;

public class Transition {
    private DcMotorEx transitionMotor;

    public Transition(HardwareMap hwMap) {
        transitionMotor = hwMap.get(DcMotorEx.class, "transitionMotor");
        transitionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTransitionMotorPower(double speed) {
        transitionMotor.setPower(speed);
    }

    public class transitionUp implements Action {
Clock timer = Clock.systemDefaultZone();
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            setTransitionMotorPower(-1);
            return false;
        }
    }
}
