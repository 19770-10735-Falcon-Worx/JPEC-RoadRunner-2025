package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

@TeleOp
public class exampleDrive extends OpMode {

    private MecanumDrive m_drivetrain;
    private ThreeDeadWheelLocalizer m_localizer;
//    private OctoQuad m_OctoQuad;
//    private HuskyLens m_HuskyLens;

    @Override
    public void init() {
        m_drivetrain = new MecanumDrive(this.hardwareMap, new Pose2d(new Vector2d(0d, 0d), 0d));
        m_localizer = (ThreeDeadWheelLocalizer) m_drivetrain.localizer;
//        m_OctoQuad = hardwareMap.get(OctoQuad.class, "octoquad");
//        m_HuskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

//        m_HuskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    @Override
    public void loop() {
        m_localizer.update();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x
        );

        double yaw = m_drivetrain.lazyImu.get().getRobotYawPitchRollAngles().getYaw();
        Pose2d heading = new Pose2d(new Vector2d(0, 0), Math.toRadians(yaw));

        input = heading.heading.times(input);

        if (gamepad1.share) m_drivetrain.lazyImu.get().resetYaw();

        m_drivetrain.setDrivePowers(new PoseVelocity2d(input, gamepad1.right_stick_x));

        telemetry.addData("Rotation", Math.toDegrees(heading.heading.toDouble()));
//        telemetry.addData("HuskyLens", m_HuskyLens.getConnectionInfo());
        telemetry.addData("Par0-Pos", m_localizer.par0.getPositionAndVelocity().position);
        telemetry.addData("Par1-Pos", m_localizer.par1.getPositionAndVelocity().position);
        telemetry.addData("perp-Pos", m_localizer.perp.getPositionAndVelocity().position);
    }
}
