package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class motorFullSpeed extends OpMode {
    DcMotorEx motor, motor2;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        //motor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(){

            motor.setPower(1.0);
            telemetry.addData("actual rpm", motor.getVelocity()/ 28 * 60);
    }
}
