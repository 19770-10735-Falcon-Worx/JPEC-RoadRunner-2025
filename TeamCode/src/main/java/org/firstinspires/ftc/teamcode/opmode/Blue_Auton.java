package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard1;


@Autonomous

public class Blue_Auton extends OpMode {

    ProgrammingBoard1 board = new ProgrammingBoard1();
    private MecanumDrive m_drivetrain;
    private ThreeDeadWheelLocalizer m_localizer;
    private double targetPosition;

    @Override
    public void init() {
        board.init(hardwareMap);
        m_drivetrain = new MecanumDrive(this.hardwareMap, new Pose2d(new Vector2d(0d, 0d), 0d));
        m_localizer = (ThreeDeadWheelLocalizer) m_drivetrain.localizer;
        targetPosition = 0;
    }

    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void loop() {
        telemetry.addData("runtime (for testing purposes)", getRuntime());
        Vector2d input = new Vector2d(0, 0);
        double yaw = -m_drivetrain.lazyImu.get().getRobotYawPitchRollAngles().getYaw();
        Pose2d heading = new Pose2d(new Vector2d(0, 0), Math.toRadians(yaw));
        input = heading.heading.times(input);
        if (getRuntime() < 0.8) {
            m_drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.5, 0), 0));
        } else {
            m_drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
        if (getRuntime() <= 4.5) {
            board.setShooterRightSpeed(1500);
            board.setShooterLeftSpeed(1500);
        } else if (getRuntime() <= 20) {
            board.setShooterRightSpeed(1500);
            board.setShooterLeftSpeed(1500);
        } else {
            board.setShooterRightSpeed(0);
            board.setShooterLeftSpeed(0);
        }
        if (getRuntime() >= 3.5 && getRuntime() <= 4) {
            board.setTransitionMotorPower(-1);
        }
        if (getRuntime() >= 4.5 && getRuntime() <= 5) {
            board.setTransitionMotorPower(0);
        }
        if (getRuntime() >= 5.0 && getRuntime() <= 6.9) {
            targetPosition = 0.666667;
        }
        if (getRuntime() >= 6.9 && getRuntime() <= 8) {
            board.setTransitionMotorPower(-1);
        }
        if (getRuntime() >= 8 && getRuntime() <= 9) {
            board.setTransitionMotorPower(0);
        }
        if (getRuntime() >= 9.0 && getRuntime() <= 11) {
            targetPosition = 0.3333333;
        }
        if (getRuntime() >= 11 && getRuntime() <= 12) {
            board.setTransitionMotorPower(-1);
        }
        if (getRuntime() >= 12 && getRuntime() <= 13) {
            board.setTransitionMotorPower(0);
        }
        if (getRuntime() >= 14 && getRuntime() <= 14.7) {
            m_drivetrain.setDrivePowers(new PoseVelocity2d(input, 0.5));
        }
        if (getRuntime() >= 14.7 && getRuntime() <= 15) {
            m_drivetrain.setDrivePowers(new PoseVelocity2d(input, 0));
            m_drivetrain.lazyImu.get().resetYaw();
        }
        if(getRuntime() >= 15 && getRuntime() <= 17) {
            m_drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -0.5), 0));
        }
        if(getRuntime() >= 17 && getRuntime() <= 19) {
            m_drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-.5, 0), 0));
        }
        if(getRuntime() >= 19 && getRuntime() <= 20) {
            m_drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
        double servoPower = board.getServoError(targetPosition) * 5;
        telemetry.addData("servo power", servoPower);
        board.setServoPower(servoPower);


    }
}
