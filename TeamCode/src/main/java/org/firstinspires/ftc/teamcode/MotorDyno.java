    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.HardwareMap;

    import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

    @TeleOp
    public class MotorDyno extends OpMode {

        DcMotorEx motor, motor2;
        double rpm = 100;
        double ticks = 0.0;
        boolean on = false, lastUp = false, lastDown = false, lastBumper = false;



        @Override
        public void init(){
            motor = hardwareMap.get(DcMotorEx.class, "shooter");
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            //ticks = motor.getMotorType().getTicksPerRev();
            ticks = 28;
        }

        @Override
        public void loop(){
            if(gamepad1.left_bumper){
                on = !on;
            }
            if (on) {
                setMotorSpeed(rpm);
            }
            else{
                motor.setVelocity(0);
            }

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean bumper = gamepad1.left_bumper;
            boolean rtBumper = gamepad1.right_bumper;

            if (up && !lastUp)         rpm += 100;
            if (down && !lastDown)     rpm -= 100;
            if (bumper && !lastBumper) on = !on;

            lastUp = up;
            lastDown = down;
            lastBumper = bumper;


            telemetry.addData("Ticks per rot", ticks);
            telemetry.addData("rpm setting:", rpm);
            telemetry.addData("motor1 velocity", motor.getVelocity()/ticks*60);
            telemetry.addData("motor2 velocity", motor2.getVelocity()/ticks*60);
        }

        public void setMotorSpeed(double rpm){

            motor.setVelocity(rpm/60*ticks);
            motor2.setVelocity(rpm/60*ticks);
        }


    }
