package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard1;


@Autonomous
// TODO: Add a button that runs during init (to change alliance) (ask pratt/nathan/westley for more info)
public class Move_Forward extends OpMode {

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

    }
}
