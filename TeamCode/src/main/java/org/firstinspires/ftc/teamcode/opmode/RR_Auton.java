package org.firstinspires.ftc.teamcode.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard1;

@Autonomous

public class RR_Auton extends OpMode {
    ProgrammingBoard1 board = new ProgrammingBoard1();
    private MecanumDrive m_drivetrain;
    private ThreeDeadWheelLocalizer m_localizer;
    private double targetPosition;
    @Override
    public void init() {
        board.init(hardwareMap);
        m_drivetrain = new MecanumDrive(this.hardwareMap, new Pose2d(new Vector2d(0d, 0d), 0d));
        m_localizer = (ThreeDeadWheelLocalizer) m_drivetrain.localizer;
    }

    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void loop() {

    }

    public class Shoot_action implements Action {
        private boolean shooterOn;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!shooterOn){
                board.setShooterLeftSpeed(1500);
                board.setShooterRightSpeed(1500);
                shooterOn = true;
            }
            board.setTransitionMotorPower(-1);
            double servoPower = board.getServoError(targetPosition) * 5;
            board.setServoPower(servoPower);
            return false;
        }
    }
}
