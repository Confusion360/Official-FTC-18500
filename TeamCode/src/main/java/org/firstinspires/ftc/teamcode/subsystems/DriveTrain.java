package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * DriveTrain.java Summary
 * - DriveTrain - constructor
 * - update - checks inputs, does all the calculations and movement stuff
 * */

public class DriveTrain extends SubsystemBase {

	private final DcMotor backRight;
	private final DcMotor backLeft;
	private final DcMotor frontRight;
	private final DcMotor frontLeft;
	private final GamepadEx gamepad1;
	IMU imu;

	public DriveTrain(final IMU imu1, GamepadEx gamepad, HardwareMap hMap, final String br, final String bl, final String fr, final String fl) {
		backRight = hMap.get(DcMotor.class, br);
		backLeft = hMap.get(DcMotor.class, bl);
		frontRight = hMap.get(DcMotor.class, fr);
		frontLeft = hMap.get(DcMotor.class, fl);
		gamepad1 = gamepad;
		imu = imu1;

		// Reverse the right side motors. This may be wrong for your setup.
		// If your robot moves backwards when commanded to go forwards,
		// reverse the left side instead.
		// See the note about this earlier on this page.
		// (copied this part from General.java, wasnt sure if relevant)
		frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
		backRight.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	public void update() {	//checks gamepad and does movement stuff
		double y = gamepad1.getLeftY(); // Remember, Y stick value is reversed
		double x = -gamepad1.getLeftX();
		double rx = -gamepad1.getRightX();

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

		frontLeft.setPower(frontLeftPower);
		backLeft.setPower(backLeftPower);
		frontRight.setPower(frontRightPower);
		backRight.setPower(backRightPower);
	}
}
