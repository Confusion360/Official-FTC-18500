package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class VerticalSlides extends SubsystemBase{
	private final DcMotor motor;

	int SLIDE_MIN_POSITION = 0;
	int SLIDE_MAX_POSITION = 2250;

	public VerticalSlides (final HardwareMap hMap, final String name) {
		motor = hMap.get(DcMotor.class, name);
	}

	public void extend () {		//extends arm to max pos
		motor.setTargetPosition(SLIDE_MAX_POSITION);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setPower(0.5);
	}

	public void reduce () {		//reduces arm to min pos
		motor.setTargetPosition(SLIDE_MIN_POSITION);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setPower(0.5);
	}

	public void moveToPos (int pos) {
		motor.setTargetPosition(pos);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setPower(0.5);
	}

	public double getTargetPos () {		//for telemetry
		return motor.getTargetPosition();
	}

	public double getCurrentPos () {	//for telemetry
		return motor.getCurrentPosition();
	}
}
