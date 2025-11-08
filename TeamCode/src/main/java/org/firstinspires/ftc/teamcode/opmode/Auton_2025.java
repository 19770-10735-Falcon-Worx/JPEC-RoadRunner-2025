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

public class Auton_2025 extends OpMode {

    ProgrammingBoard1 board = new ProgrammingBoard1();
    private MecanumDrive m_drivetrain;
    private ThreeDeadWheelLocalizer m_localizer;

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
      if (getRuntime() < 3){
          m_drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 1), 0));
      } else {
          m_drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
      telemetry.addData("runtime", getRuntime());
    }
}
