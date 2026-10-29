package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    private final DcMotorEx shooter;

    public Shooter(HardwareMap hwMap) {
        PIDFCoefficients newPID = new PIDFCoefficients(10,5,0, 3);
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void setPIDF(PIDFCoefficients cof) {
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, cof);

    }
    public PIDFCoefficients getPIDValues() {
        return shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setShooterSpeed(double speed) {
        shooter.setVelocity((speed / 60) * 28);
    }

    public double getShooterSpeed() {
        return (shooter.getVelocity() * 60) / 28;
    }
    public SequentialAction runShooter() {
        return new SequentialAction(
                new InstantAction(() -> setShooterSpeed(3500))
        );
    }

    public SequentialAction stopShooter() {
        return new SequentialAction(
                new InstantAction(() -> setShooterSpeed(0))
        );
    }
}

