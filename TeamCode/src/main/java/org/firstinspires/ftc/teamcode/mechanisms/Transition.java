package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transition {
    private final DcMotorEx transitionMotor;

    public Transition(HardwareMap hwMap) {
        transitionMotor = hwMap.get(DcMotorEx.class, "transitionMotor");
        transitionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTransitionMotorPower(double speed) {
        transitionMotor.setPower(speed);
    }

    // TODO: Create an action to hold the motor in reverse so we can clear jams

    public SequentialAction runTransition() {
        return new SequentialAction(
                new InstantAction(()->setTransitionMotorPower(-1)),
                new SleepAction(1.0),
                new InstantAction(()->setTransitionMotorPower(0))
        );
    }
}
