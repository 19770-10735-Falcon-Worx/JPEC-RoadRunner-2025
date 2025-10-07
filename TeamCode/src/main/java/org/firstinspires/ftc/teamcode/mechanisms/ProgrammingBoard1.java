package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ProgrammingBoard1 {

   // private DigitalChannel touchSensor;
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
   // private DcMotorEx motor_3;
    private double ticksPerRotation;
   // private Servo servo;
   // private AnalogInput pot;
   // private ColorSensor colorSensor;
   // private DistanceSensor distanceSensor;
   // private IMU imu;

    public void init(HardwareMap hwMap) {
        //touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
       // touchSensor.setMode(DigitalChannel.Mode.INPUT);
        shooterLeft = hwMap.get(DcMotorEx.class, "shooterLeft");
        shooterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = shooterLeft.getMotorType().getTicksPerRev();
        shooterRight = hwMap.get(DcMotorEx.class, "shooterRight");
        shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = shooterRight.getMotorType().getTicksPerRev();
       // motor_3 = hwMap.get(DcMotorEx.class, "motor 3");
       // motor_3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
       // ticksPerRotation = motor_3.getMotorType().getTicksPerRev();
       // servo = hwMap.get(Servo.class, "servo");
       // pot = hwMap.get(AnalogInput.class, "pot");

       // colorSensor = hwMap.get(ColorSensor.class, "sensor_color_distance");
       // distanceSensor = hwMap.get(DistanceSensor.class, "sensor_color_distance");
       // imu = hwMap.get(IMU.class, "imu");

       // RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
       // imu.initialize(new IMU.Parameters(RevOrientation));
    }

    //public boolean getTouchSensorState() {
      //  return !touchSensor.getState();
    //}

    public void setShooterLeftSpeed(double speed) {
        shooterLeft.setVelocity(-((speed/60)*28));
    }

    public void setShooterRightSpeed(double speed) {
        shooterRight.setVelocity(((speed/60)*28));
    }

    //public void setMotor3Speed(double speed) {
      //  motor_3.setPower(speed);
    //}

    public double getShooterLeftRotations() {
        return shooterLeft.getCurrentPosition() / ticksPerRotation;
    }
    public double getShooterLeftSpeed() {
        return (-(shooterLeft.getVelocity()/28)*60);
    }

    public double getShooterRightRotations() {
        return shooterRight.getCurrentPosition() / ticksPerRotation;
    }
    public double getShooterRightSpeed() {
        return ((shooterRight.getVelocity()/28)*60);

    }

    //public double getMotor3Rotations() {
      //  return motor_3.getCurrentPosition() / ticksPerRotation;
    //}


    //public void setServoPosition(double position) {
       // servo.setPosition(position);
    //}

   // public double getPotAngle() {
       // return Range.scale(pot.getVoltage(), 0, pot.getMaxVoltage(), 0, 270);
   // }

   // public int getAmountRed() {
       // return colorSensor.red();
   // }

    //public int getAmountBlue() {
      //  return colorSensor.blue();
   // }

   // public int getAmountGreen() {
    //    return colorSensor.green();
   // }

    //public double getDistance(DistanceUnit du) {
     //   return distanceSensor.getDistance(du);
    //}

   // public double getYaw(AngleUnit angleUnit) {
   //     return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
   // }

  //  public double getPitch(AngleUnit angleUnit) {
   //     return imu.getRobotYawPitchRollAngles().getPitch(angleUnit);
   // }

  //  public double getRoll(AngleUnit angleUnit) {
  //      return imu.getRobotYawPitchRollAngles().getRoll(angleUnit);
  //  }


}