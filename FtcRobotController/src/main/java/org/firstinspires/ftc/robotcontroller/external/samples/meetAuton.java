package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class meetAuton extends LinearOpMode {
    private DcMotor shooter, transition, loader, leftFront, rightFront, leftBack, rightBack;

    public void init(HardwareMap hwMap) {
        shooter = hwMap.get(DcMotor.class, "shooter");
        shooter.setMode(DcMotor. RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        transition = hwMap.get(DcMotor.class, "transition");
        loader = hwMap.get(DcMotor.class, "loader");
        leftFront = hwMap.get(DcMotor.class,"leftFront" );
        rightBack = hwMap.get(DcMotor.class,"rightBack" );
        leftBack = hwMap.get(DcMotor.class,"leftBack" );
        rightFront = hwMap.get(DcMotor.class,"rightFront" );
    }

    @Override
    public void runOpMode() {
        init(hardwareMap);
        waitForStart();
        shooter.setPower(0.75);
        sleep(1000);
        transition.setPower(0.5);
        loader.setPower(0.5);
        sleep(5000);
        shooter.setPower(0.0);
        transition.setPower(0.0);
        loader.setPower(0.0);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        sleep(2000);
    }
}
