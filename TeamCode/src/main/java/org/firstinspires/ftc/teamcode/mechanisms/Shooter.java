package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    private final DcMotorEx shooter;

    public Shooter(HardwareMap hwMap) {
        this(hwMap, "shooter");
    }

    public Shooter(HardwareMap hwMap, String motorName) {
        shooter = hwMap.get(DcMotorEx.class, motorName);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setPIDF(PIDFCoefficients cof) {
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, cof);

    }
    public PIDFCoefficients getPIDValues() {
        return shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDirectionReverse(){
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
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

