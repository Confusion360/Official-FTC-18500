package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class General2024 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //position for the slide max and min, idk the max value check that with someone
        int SLIDE_MIN_POSITION = 0;
        //I calculated revolutions using circumference and slide height change
        //I calculated tics per revolution by PPR
        int SLIDE_MAX_POSITION = -5600;

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        // Motor for linear slide
        DcMotor horizontalSlides = hardwareMap.dcMotor.get("hslide");

        //servo for claw
        Servo intakeServo = hardwareMap.servo.get("intakeservo");
        Servo intakeHinge = hardwareMap.servo.get("intakehinge");

        Servo claw_tilt = hardwareMap.servo.get("ctilt");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset the motor encoder so that it reads zero ticks
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the arm to the down position
        horizontalSlides.setTargetPosition(SLIDE_MIN_POSITION);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            intakeServo.setPosition(1);
            intakeHinge.setPosition(0);
            //ground position is right
            claw_tilt.setPosition(1);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // If the A button is pressed, intake spins inwards
            if (gamepad1.right_bumper) {
                horizontalSlides.setTargetPosition(SLIDE_MAX_POSITION);
                horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalSlides.setPower(0.5);
            }

            // If the B button is pressed, lower the arm
            if (gamepad1.left_bumper) {
                horizontalSlides.setTargetPosition(SLIDE_MIN_POSITION);
                horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalSlides.setPower(0.5);
            }

            if (gamepad1.a && intakeHinge.getPosition() == 0){
                intakeHinge.setPosition(1);
            } else if (gamepad1.a && intakeHinge.getPosition() == 1) {
                intakeHinge.setPosition(0);
            }

            if (gamepad1.b && intakeServo.getPosition() == 0 || intakeServo.getPosition() == 1.5){
                // turns inwards
                intakeServo.setPosition(1);
            }else if (gamepad1.b && intakeServo.getPosition() == 1){
                // stops turning
                intakeServo.setPosition(0.5);
            }else if (gamepad1.x && intakeServo.getPosition() == 1 || intakeServo.getPosition() == 1.5){
                // turns outwards
                intakeServo.setPosition(0);
            }else if (gamepad1.x && intakeServo.getPosition() == 0){
                // stops turning
                intakeServo.setPosition(0.5);
            }


            // Get the current position of the armMotor
            double position = horizontalSlides.getCurrentPosition();

            // Get the target position of the armMotor
            double desiredPosition = horizontalSlides.getTargetPosition();

            // Show the position of the armMotor on telemetry
            telemetry.addData("Encoder Position", position);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Desired Position", desiredPosition);

            telemetry.update();
        }
    }
}