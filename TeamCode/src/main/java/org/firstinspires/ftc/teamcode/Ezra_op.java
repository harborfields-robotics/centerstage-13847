/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Ezra_op", group="Linear OpMode")
public class Ezra_op extends LinearOpMode
{

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    public Servo clawRight;
    public Servo clawLeft;
    private Servo wrist;
    public static final double MAX_POSITION = 6000, MIN_POSITION = 0;
    private Hardware hardware;

    private int armPosition = 0;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        hardware = new Hardware(hardwareMap);
        arm = hardwareMap.get(DcMotor.class, "ARM");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
        clawRight = hardwareMap.get(Servo.class, "CLAWR");
        clawLeft = hardwareMap.get(Servo.class, "CLAWL");
        wrist = hardwareMap.get(Servo.class, "WRIST");

        //claw_Green.scaleRange(0.25, 0.75);
        //elbow_Left.scaleRange(0,0.25);  servo programs

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean slowMode = false;
        boolean armSlowMode = false;

        while (opModeIsActive()) {


            // run until the end of the match (driver presses STOP)
            //while (opModeIsActive()) {

                double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                if (gamepad1.left_bumper)
                    slowMode = !slowMode;
                if (gamepad2.a)
                    armSlowMode = !armSlowMode;

                // Send calculated power to wheels
                double powers[] = {leftFrontPower, leftBackPower, rightBackPower, rightFrontPower};
                if (slowMode)
                    hardware.setMotorSlowMode(powers);
                else
                    hardware.setMotorPowers(powers);

                if (gamepad1.dpad_left){
                    hardware.turnLeft(45,1);
                }
                else if (gamepad1.dpad_right){
                    hardware.turnRight(45,1);
                }

                /*
                double armPower = 0;
                if (armSlowMode)
                    hardware.setArmsSlowMode(armPower);
                else
                    hardware.setArmPower(armPower); */
                if (Math.abs(gamepad2.right_stick_y) > 0.2)
                    arm.setTargetPosition(armPosition = Range.clip(Math.round(armPosition + gamepad2.right_stick_y * 5), 0, Integer.MAX_VALUE));

                if (gamepad2.left_bumper)
                    hardware.setClawLeftPositon(1);
                else if (gamepad2.left_trigger > 0.2)
                    hardware.setClawLeftPositon(hardware.getClawLeftposition() - 0.05);
                else hardware.setClawLeftPositon(0.5);

                if (gamepad2.right_bumper)
                    hardware.setClawRightPosition(1);
                else if (gamepad2.right_trigger > 0.2)
                    hardware.setClawRightPosition(hardware.getClawRightposition() - 0.05);
                else hardware.setClawRightPosition(0.5);

                if (gamepad2.right_stick_y > 0.2){
                    hardware.setWristPosition(hardware.getWristposition() + 0.05);
                }
                else if (gamepad2.right_stick_y < -0.2){
                    hardware.setWristPosition(hardware.getWristposition() - 0.05);
                }
                else{
                    hardware.setWristPosition(0.5);
                }

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();
        }
    }
}


