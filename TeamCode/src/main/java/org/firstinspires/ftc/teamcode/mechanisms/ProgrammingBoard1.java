package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is a class where all of the hardware exists, like motors or servos.
 * @Hardware: Motors, servos, encoders
 */
public class ProgrammingBoard1 {


    private DcMotorEx shooterLeft, shooterRight, intake, transitionMotor;


    private double ticksPerRotation;
    private CRServo turntable;


    public void init(HardwareMap hwMap) {

       shooterLeft = hwMap.get(DcMotorEx.class, "shooterLeft");
        shooterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = shooterLeft.getMotorType().getTicksPerRev();
        shooterRight = hwMap.get(DcMotorEx.class, "shooterRight");
        shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = shooterRight.getMotorType().getTicksPerRev();

        transitionMotor = hwMap.get(DcMotorEx.class, "transitionMotor");
        transitionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        turntable = hwMap.get(CRServo.class, "turntable");

    }



    public void setShooterLeftSpeed(double speed) {
        shooterLeft.setVelocity(-((speed / 60) * 28));
    }

    public void setShooterRightSpeed(double speed) {
        shooterRight.setVelocity(((speed / 60) * 28));
    }
    public void setIntakePower(double speed) {
        intake.setPower(speed);
    }
    public void setTransitionMotorPower(double speed) {
        transitionMotor.setPower(speed);
    }

    public double getShooterLeftRotations() {
        return shooterLeft.getCurrentPosition() / ticksPerRotation;
    }

    public double getShooterLeftSpeed() {
        return (-(shooterLeft.getVelocity() / 28) * 60);
    }

    public double getShooterRightRotations() {
        return shooterRight.getCurrentPosition() / ticksPerRotation;
    }

    public double getShooterRightSpeed() {
        return ((shooterRight.getVelocity() / 28) * 60);

    }


    public double getTurntablePosition() {

        if (intake.getCurrentPosition() < 0) {
            return 1 - Math.abs((intake.getCurrentPosition() / 8192.0) % 1);
        }
        return (intake.getCurrentPosition() / 8192.0) % 1;
    }

    public void setServoPower(double power) {
        turntable.setPower(power);
    }

    public double getServoError(double target) {
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