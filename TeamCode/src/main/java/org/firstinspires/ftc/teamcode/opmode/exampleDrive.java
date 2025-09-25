package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class exampleDrive extends OpMode {

    private MecanumDrive m_drivetrain;
    private Localizer m_localizer;

    @Override
    public void init() {
        m_drivetrain = new MecanumDrive(this.hardwareMap, new Pose2d(new Vector2d(0d, 0d), 0d));
        m_localizer = m_drivetrain.localizer;

    }

    @Override
    public void loop() {
        m_localizer.update();

        Vector2d input = new Vector2d(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x
        );

        input = m_localizer.getPose().heading.inverse().times(input);

        m_drivetrain.setDrivePowers(new PoseVelocity2d(input, gamepad1.right_stick_x));

        telemetry.addData("Rotation", m_localizer.getPose().heading);
    }
}
