package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.time.Clock;
import java.time.LocalDateTime;
import java.util.Timer;

public class Shooter {
    private DcMotorEx shooterLeft, shooterRight;

    public Shooter(HardwareMap hwMap) {
        shooterLeft = hwMap.get(DcMotorEx.class, "shooterLeft");
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight = hwMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setShooterLeftSpeed(double speed) {
        shooterLeft.setVelocity(-((speed / 60) * 28));
    }

    public void setShooterRightSpeed(double speed) {
        shooterRight.setVelocity(((speed / 60) * 28));
    }

    public SequentialAction runShooter() {
        return new SequentialAction(
                new InstantAction(() -> setShooterLeftSpeed(3500)),
                new InstantAction(() -> setShooterRightSpeed(800))
        );
    }

    public SequentialAction stopShooter() {
        return new SequentialAction(
                new InstantAction(() -> setShooterLeftSpeed(0)),
                new InstantAction(() -> setShooterRightSpeed(0))
        );
    }
}

