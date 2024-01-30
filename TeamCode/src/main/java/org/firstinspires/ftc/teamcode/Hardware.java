package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


public class Hardware {
        // Declare OpMode members for each of the 4 motors.
    // public static final double ticks_per_tile = 23 / (96 / 25.4) * 384.5;
    static int ticks_per_tile = 620; // constant for strafe methods
    static double strafe_constant = 1.3;
    static int ticks_per_side_difference = 30;
    static double ticks_per_degree = 7.45;

    private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor rightBackDrive = null;
        private DcMotor arm = null;
        private Servo clawRight;
        private Servo clawLeft;
        private Servo wrist;
        private IMU imu;
        private double yawChangeAmt = 10;
        private double imuangle;

        private DcMotor FL, BL, BR, FR;
        public static final double SLOW_RATE = 0.3;

        public Hardware(HardwareMap hardwareMap) {
            FL = leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
            BL = leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
            FR = rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
            BR = rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
            arm = hardwareMap.get(DcMotor.class, "ARM");
            clawRight = hardwareMap.get(Servo.class, "CLAWR");
            clawLeft = hardwareMap.get(Servo.class, "CLAWL");
            wrist = hardwareMap.get(Servo.class, "WRIST");
            imu =  hardwareMap.get(IMU.class, "imu");

            arm.setMode(STOP_AND_RESET_ENCODER);
            arm.setMode(RUN_WITHOUT_ENCODER);

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            arm.setDirection(DcMotor.Direction.REVERSE);

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            // Now initialize the IMU with this mounting orientation
            // Note: if you choose two conflicting directions, this initialization will cause a code exception.
            imu.initialize(new IMU.Parameters(orientationOnRobot));
        }

        public double getYaw() {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            return orientation.getYaw(AngleUnit.DEGREES);
        }

        public void storeImu() {
            imuangle = getYaw();
        }

        public void checkImu() {

            if (Math.abs(getYaw() - imuangle) < yawChangeAmt)
                ;
        }

        public void setArmPower(double power)
        {
            arm.setPower(power);
        }

        public void setClawRightPosition(double power){clawRight.setPosition(power);}
        public double getClawRightposition() { return clawRight.getPosition(); }

        public void setClawLeftPositon(double power){clawLeft.setPosition(power);}
        public double getClawLeftposition() { return clawLeft.getPosition(); }

        public void setWristPosition(double power){wrist.setPosition(power);}
        public double getWristposition() { return wrist.getPosition(); }




    public void setMotorPowers(double... powers)
        {
            leftFrontDrive.setPower(powers[0]);
            leftBackDrive.setPower(powers[1]);
            rightBackDrive.setPower(powers[2]);
            rightFrontDrive.setPower(powers[3]);
        }

        public void setMotorSlowMode(double... powers)
            {
                leftFrontDrive.setPower(powers[0] * SLOW_RATE);
                leftBackDrive.setPower(powers[1] * SLOW_RATE);
                rightBackDrive.setPower(powers[2] * SLOW_RATE);
                rightFrontDrive.setPower(powers[3] * SLOW_RATE);
        }
        public void setArmsSlowMode(double power)
    {
        arm.setPower( power * SLOW_RATE);
    }


        public void setMotorPower(double fl, double fr, double bl, double br)
        {
            leftFrontDrive.setPower(fl);
            rightFrontDrive.setPower(fr);
            leftBackDrive.setPower(bl);
            rightBackDrive.setPower(br);
        }

        public void setMotorTargets(double fl, double fr, double bl, double br)
        {
            leftFrontDrive.setTargetPosition((int) fl);
            rightFrontDrive.setTargetPosition((int) fr);
            leftBackDrive.setTargetPosition((int) bl);
            rightBackDrive.setTargetPosition((int) br);
        }

        public void setMotorModes(DcMotor.RunMode mode) {
            leftFrontDrive.setMode(mode);
            rightFrontDrive.setMode(mode);
            leftBackDrive.setMode(mode);
            rightBackDrive.setMode(mode);
        }

        public void strafeLeft(double num_tiles, double power) {
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorTargets((int) (-num_tiles * ticks_per_tile * strafe_constant), // FL
                (int) (num_tiles  * ticks_per_tile * strafe_constant), // FR
                (int) (num_tiles  * ticks_per_tile * strafe_constant), // BL
                (int) (-num_tiles * ticks_per_tile * strafe_constant)); // BR
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower(power, power, power, power);
        while (FL.getCurrentPosition() > FL.getTargetPosition() && BL.getCurrentPosition() < BL.getTargetPosition() && FR.getCurrentPosition() < FR.getTargetPosition() && BR.getCurrentPosition() > BR.getTargetPosition()) {
            telemetry.addData("Robot is strafing left...        ", 11115);
            telemetry.addData("FL target: ", FL.getTargetPosition());
            telemetry.addData("BL target: ", BL.getTargetPosition());
            telemetry.addData("FR target: ", FR.getTargetPosition());
            telemetry.addData("BR target: ", BR.getTargetPosition());
            telemetry.addData("FL position: ", FL.getCurrentPosition());
            telemetry.addData("BL position: ", BL.getCurrentPosition());
            telemetry.addData("FR position: ", FR.getCurrentPosition());
            telemetry.addData("BR position: ", BR.getCurrentPosition());
            telemetry.update();
        }
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorPower(0, 0, 0, 0);


        //drive(180, num_tiles, power);
    }

    public void strafeRight(double num_tiles, double power) {
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorTargets((int) (num_tiles  * ticks_per_tile * strafe_constant),
                (int) (-num_tiles * ticks_per_tile * strafe_constant),
                (int) (-num_tiles * ticks_per_tile * strafe_constant),
                (int) (num_tiles  * ticks_per_tile * strafe_constant));
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        while (FL.getCurrentPosition() < FL.getTargetPosition() && BL.getCurrentPosition() > BL.getTargetPosition() && FR.getCurrentPosition() > FR.getTargetPosition() && BR.getCurrentPosition() < BR.getTargetPosition()) {
            setMotorPower(power, power, power, power);
            telemetry.addData("Robot is strafing right...        ", 11115);
            telemetry.addData("FL target: ", FL.getTargetPosition());
            telemetry.addData("BL target: ", BL.getTargetPosition());
            telemetry.addData("FR target: ", FR.getTargetPosition());
            telemetry.addData("BR target: ", BR.getTargetPosition());
            telemetry.addData("FL position: ", FL.getCurrentPosition());
            telemetry.addData("BL position: ", BL.getCurrentPosition());
            telemetry.addData("FR position: ", FR.getCurrentPosition());
            telemetry.addData("BR position: ", BR.getCurrentPosition());
            telemetry.update();
        }
        setMotorPower(0, 0, 0, 0);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //drive(0, num_tiles, power);
    }

    public void driveForward(double num_tiles, double power) {
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorTargets((int) (num_tiles * ticks_per_tile),
                (int) (num_tiles * ticks_per_tile),
                (int) (num_tiles * ticks_per_tile),
                (int) (num_tiles * ticks_per_tile));
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(power, power, power, power);
        while (FL.getCurrentPosition() < FL.getTargetPosition() && BL.getCurrentPosition() < BL.getTargetPosition() && FR.getCurrentPosition() < FR.getTargetPosition() && BR.getCurrentPosition() < BR.getTargetPosition()) {
            telemetry.addData("FL target: ", FL.getTargetPosition());
            telemetry.addData("BL target: ", BL.getTargetPosition());
            telemetry.addData("FR target: ", FR.getTargetPosition());
            telemetry.addData("BR target: ", BR.getTargetPosition());
            telemetry.addData("FL position: ", FL.getCurrentPosition());
            telemetry.addData("BL position: ", BL.getCurrentPosition());
            telemetry.addData("FR position: ", FR.getCurrentPosition());
            telemetry.addData("BR position: ", BR.getCurrentPosition());
            telemetry.update();
        }
        setMotorPower(0, 0, 0, 0);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //drive(90, num_tiles, power);
    }

    public void driveBackward(double num_tiles, double power) {
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorTargets((int) (-num_tiles * ticks_per_tile),
                (int) (-num_tiles * ticks_per_tile),
                (int) (-num_tiles * ticks_per_tile),
                (int) (-num_tiles * ticks_per_tile));
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        while (FL.getCurrentPosition() > FL.getTargetPosition() && BL.getCurrentPosition() > BL.getTargetPosition() && FR.getCurrentPosition() > FR.getTargetPosition() && BR.getCurrentPosition() > BR.getTargetPosition()) {
            setMotorPower(power, power, power, power);
        }
        setMotorPower(0, 0, 0, 0);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //drive(270, num_tiles, power);
    }
    public void smoothDriveForward(double num_tiles, double power) {
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorTargets((int) (num_tiles * ticks_per_tile),
                (int) (num_tiles * ticks_per_tile),
                (int) (num_tiles * ticks_per_tile),
                (int) (num_tiles * ticks_per_tile));
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPowers(power);

    }

    public void armAuto(double power, long milliseconds) throws java.lang.InterruptedException{
            setArmPower(power);
            Thread.sleep(milliseconds);
            setArmPower(0);
        }

        public void clawRightAuto(double position, long milliseconds) throws java.lang.InterruptedException {
            setClawRightPosition(position);
            Thread.sleep(milliseconds);
            setClawRightPosition(0.5);
        }

         public void clawLeftAuto(double position, long milliseconds) throws java.lang.InterruptedException {
            setClawLeftPositon(position);
            Thread.sleep(milliseconds);
            setClawLeftPositon(0.5);
         }

        public void wristAuto(double position, long milliseconds) throws java.lang.InterruptedException {
            setWristPosition(position);
            Thread.sleep(milliseconds);
            setWristPosition(0.5);
        }


        public void EmilysCopyPaste(int power) throws java.lang.InterruptedException{
        while ((FL.getPower() < power) && (FL.getCurrentPosition() < FL.getTargetPosition())) {
            setMotorPower(FL.getPower()+(power/10), FR.getPower()+(power/10), BL.getPower()+(power/10), BR.getPower()+(power/10));
            try {
                Thread.sleep(25);
            } catch (InterruptedException e) {}
        }
        setMotorPower(power, power, power, power);

        while (FL.getCurrentPosition() < FL.getTargetPosition() - (ticks_per_tile * 0.12)) {
            telemetry.addData("FL target: ", FL.getTargetPosition());
            telemetry.addData("BL target: ", BL.getTargetPosition());
            telemetry.addData("FR target: ", FR.getTargetPosition());
            telemetry.addData("BR target: ", BR.getTargetPosition());
            telemetry.addData("FL position: ", FL.getCurrentPosition());
            telemetry.addData("BL position: ", BL.getCurrentPosition());
            telemetry.addData("FR position: ", FR.getCurrentPosition());
            telemetry.addData("BR position: ", BR.getCurrentPosition());
            telemetry.update();
        }
        setMotorPower(power, power, power, power);
        while (FL.getPower() > 0 && FL.getCurrentPosition() < FL.getTargetPosition()) {
            setMotorPower(FL.getPower()-(power/10), FR.getPower()-(power/10), BL.getPower()-(power/10), BR.getPower()-(power/10));
            try {
                Thread.sleep(25);
            } catch (InterruptedException e) {}
        }
        //
        // fixEncoderTicksDifferential(.2);
        setMotorPower(0,0,0,0);


        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorPower(0, 0, 0, 0);


        //drive(90, num_tiles, power);
    }


    // DOES NOT WORK PROPERLY
    public void drive(double angle, double num_tiles, double power) throws InterruptedException {
        // FL and BR are y+x
        // FR and BL are y-x
        double x = Math.cos(Math.toRadians(angle));
        double y = Math.sin(Math.toRadians(angle));
        setMotorModes(RUN_USING_ENCODER);
        setMotorTargets(FL.getTargetPosition() + (int)(num_tiles * y+x),
                FR.getTargetPosition() + (int)(num_tiles * y-x),
                BL.getTargetPosition() + (int)(num_tiles * y-x),
                BR.getTargetPosition() + (int)(num_tiles * y+x));
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(Math.abs(y+x), Math.abs(y-x), Math.abs(y-x), Math.abs(y+x));
        while (FL.getCurrentPosition() != FL.getTargetPosition() && BL.getCurrentPosition() != BL.getTargetPosition() && FR.getCurrentPosition() != FR.getTargetPosition() && BR.getCurrentPosition() != BR.getTargetPosition()) {
            Thread.sleep(10);
        }
        setMotorPower(0,0,0,0);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void fixEncoderTicksDifferential(double power) {
        setMotorModes(RUN_USING_ENCODER);
        setMotorTargets(FL.getTargetPosition() + ticks_per_side_difference,
                FR.getTargetPosition(),
                BL.getTargetPosition() + ticks_per_side_difference,
                BR.getTargetPosition());
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        while (FL.getCurrentPosition() != FL.getTargetPosition() && BL.getCurrentPosition() != BL.getTargetPosition() && FR.getCurrentPosition() != FR.getTargetPosition() && BR.getCurrentPosition() != BR.getTargetPosition()) {
            setMotorPower(power, power, power, power);
        }
        setMotorPower(0,0,0,0);

    }

    public void turnLeft(double degrees, double power) {
        resetMotorEncoders();
        setMotorModes(RUN_USING_ENCODER);
        setMotorTargets(FL.getCurrentPosition() - (int)(degrees * ticks_per_degree),
                FR.getCurrentPosition() + (int)(degrees * ticks_per_degree),
                BL.getCurrentPosition() - (int)(degrees * ticks_per_degree),
                BR.getCurrentPosition() + (int)(degrees * ticks_per_degree)
        );
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        while (FL.getCurrentPosition() > FL.getTargetPosition() &&
                BL.getCurrentPosition() > BL.getTargetPosition() &&
                FR.getCurrentPosition() < FR.getTargetPosition() &&
                BR.getCurrentPosition() < BR.getTargetPosition()
        ) {
            setMotorPower(power,power,power,power);
            telemetry.addData(">> turnLeft() is running...    ", 11115);
            telemetry.addData("FL target: ", FL.getTargetPosition());
            telemetry.addData("BL target: ", BL.getTargetPosition());
            telemetry.addData("FR target: ", FR.getTargetPosition());
            telemetry.addData("BR target: ", BR.getTargetPosition());
            telemetry.addData("FL position: ", FL.getCurrentPosition());
            telemetry.addData("BL position: ", BL.getCurrentPosition());
            telemetry.addData("FR position: ", FR.getCurrentPosition());
            telemetry.addData("BR position: ", BR.getCurrentPosition());
            telemetry.update();
        }
        setMotorPower(0,0,0,0);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turnRight(double degrees, double power) {
        resetMotorEncoders();
        setMotorModes(RUN_USING_ENCODER);
        setMotorTargets(FL.getCurrentPosition() + (int)(degrees * ticks_per_degree),
                FR.getCurrentPosition() - (int)(degrees * ticks_per_degree),
                BL.getCurrentPosition() + (int)(degrees * ticks_per_degree),
                BR.getCurrentPosition() - (int)(degrees * ticks_per_degree)
        );
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        while ( FL.getCurrentPosition() < FL.getTargetPosition() &&
                BL.getCurrentPosition() < BL.getTargetPosition() &&
                FR.getCurrentPosition() > FR.getTargetPosition() &&
                BR.getCurrentPosition() > BR.getTargetPosition()
        ) {
            setMotorPower(power,power,power,power);
            telemetry.addData(">> turnLeft() is running...    ", 11115);
            telemetry.addData("FL target: ", FL.getTargetPosition());
            telemetry.addData("BL target: ", BL.getTargetPosition());
            telemetry.addData("FR target: ", FR.getTargetPosition());
            telemetry.addData("BR target: ", BR.getTargetPosition());
            telemetry.addData("FL position: ", FL.getCurrentPosition());
            telemetry.addData("BL position: ", BL.getCurrentPosition());
            telemetry.addData("FR position: ", FR.getCurrentPosition());
            telemetry.addData("BR position: ", BR.getCurrentPosition());
            telemetry.update();
        }
        setMotorPower(0,0,0,0);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetMotorEncoders() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}