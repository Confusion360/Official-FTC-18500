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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HorizontalSlides extends SubsystemBase {
	private final Servo rightServo;
	private final Servo leftServo;
	double rightOffset = 0.0;

	public HorizontalSlides (final HardwareMap hMap, final String rServoName, final String lServoName) {
		rightServo = hMap.get(Servo.class, rServoName);
		leftServo = hMap.get(Servo.class, lServoName);

	}

	public void move(double pos) {
		rightServo.setPosition(pos + rightOffset);
		leftServo.setPosition(1-pos);
	}

	public void move(double pos, double offset) {
		rightServo.setPosition(pos + offset);
		leftServo.setPosition(1-pos);
	}
}
