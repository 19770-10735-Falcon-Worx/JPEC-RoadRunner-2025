package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanisms.PIDConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Autonomous
public class testAuton extends LinearOpMode {
    private DcMotor transition, loader, leftFront, rightFront, leftBack, rightBack;
    private Shooter m_shooter, m_shooter2;
    double shooterRPM = 4100.0;

    public void init(HardwareMap hwMap){
        m_shooter = new Shooter(this.hardwareMap, "shooter");
        m_shooter2 = new Shooter(this.hardwareMap, "shooter2");
        m_shooter2.setDirectionReverse();
        transition = hwMap.get(DcMotor.class, "transition");
        transition.setDirection(DcMotorSimple.Direction.REVERSE);
        loader = hwMap.get(DcMotor.class, "loader");
        loader.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients newPID = new PIDFCoefficients(PIDConstants.P_VALUE, PIDConstants.I_VALUE, PIDConstants.D_VALUE, PIDConstants.F_VALUE);
        m_shooter.setPIDF(newPID);
        m_shooter2.setPIDF(newPID);
    }

    @Override
    public void runOpMode(){
        init(hardwareMap);
        waitForStart();
        m_shooter.setShooterSpeed(shooterRPM);
        m_shooter2.setShooterSpeed(shooterRPM);
        sleep(15000);
        transition.setPower(1.0);
        sleep(200);
        loader.setPower(1.0);
        sleep(6000);
        m_shooter.setShooterSpeed(0);
        m_shooter2.setShooterSpeed(0);
        transition.setPower(0.0);
        loader.setPower(0.0);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        sleep(500);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
    }
}
