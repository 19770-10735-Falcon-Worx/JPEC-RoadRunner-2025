package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Loader {
    private final DcMotorEx loaderMotor;

    public Loader(HardwareMap hwMap) {
        loaderMotor = hwMap.get(DcMotorEx.class, "loader");
        loaderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLoaderMotorPower(double speed) {
        loaderMotor.setPower(speed);
    }

    // Runs the loader backward briefly to clear a jam
    public SequentialAction reverseLoader(double seconds) {
        return new SequentialAction(
                new InstantAction(() -> setLoaderMotorPower(1)),
                new SleepAction(seconds),
                new InstantAction(() -> setLoaderMotorPower(0))
        );
    }

    public SequentialAction runLoader() {
        return new SequentialAction(
                new InstantAction(() -> setLoaderMotorPower(-1)),
                new SleepAction(1.0),
                new InstantAction(() -> setLoaderMotorPower(0))
        );
    }
}