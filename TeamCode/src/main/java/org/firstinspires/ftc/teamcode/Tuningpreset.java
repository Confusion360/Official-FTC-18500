package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Tuningpreset extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {

		//servos for arms
		Servo arm_l = hardwareMap.servo.get("al");
		Servo arm_r = hardwareMap.servo.get("ar");


		// set position of arm servos
		arm_l.setPosition(0.5);
		arm_r.setPosition(0.5 - 0.05);  //middles are slightly different, tune this later\

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
			telemetry.addData("left pos:", arm_l.getPosition());
			telemetry.addData("right pos:", arm_r.getPosition());

			if (gamepad1.a) {
				arm_l.setPosition(arm_l.getPosition()+0.01);
			}
			if (gamepad1.b) {
				arm_r.setPosition((arm_r.getPosition()+0.01));
			}
			telemetry.addData("difference (r-l):", arm_r.getPosition() - arm_l.getPosition());
		}
	}
}