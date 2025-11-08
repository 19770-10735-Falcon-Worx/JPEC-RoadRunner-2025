package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard1;

@TeleOp

public class Orion extends OpMode {
    double hehe;
    ProgrammingBoard1 board = new ProgrammingBoard1();

    @Override
    public void init() {
        board.init(hardwareMap);
    }


    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

        resetRuntime();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            hehe = 2;
        } else {
            hehe = 1;
        }
        board.setServoPower((-gamepad1.left_stick_y / 2) * hehe);
    }
}
