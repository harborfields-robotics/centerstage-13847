package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Hardware {
        // Declare OpMode members for each of the 4 motors.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor rightBackDrive = null;
        private DcMotor slideLeft = null;
        private DcMotor slideRight = null;
        private IMU imu;
        private double yawChangeAmt = 10;
        private double imuangle;

        public Hardware(HardwareMap hardwareMap) {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
            leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
            rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
            slideRight = hardwareMap.get(DcMotor.class, "SR");
            imu =  hardwareMap.get(IMU.class, "imu");

            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            slideLeft.setDirection(DcMotor.Direction.FORWARD);
            slideRight.setDirection(DcMotor.Direction.FORWARD);


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

        public void setSlidesPower(double power)
        {
            slideLeft.setPower(power);
        }

        public void setMotorPowers(double... powers)
        {
            leftFrontDrive.setPower(powers[0]);
            leftBackDrive.setPower(powers[1]);
            rightBackDrive.setPower(powers[2]);
            rightFrontDrive.setPower(powers[3]);
        }

        /*

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral =  gamepad1.left_stick_x;
                double yaw     =  gamepad1.right_stick_x;

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower  = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower   = axial - lateral + yaw;
                double rightBackPower  = axial + lateral - yaw;

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                    leftBackPower   /= max;
                    rightBackPower  /= max;
                }


                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();
            }
         */
}