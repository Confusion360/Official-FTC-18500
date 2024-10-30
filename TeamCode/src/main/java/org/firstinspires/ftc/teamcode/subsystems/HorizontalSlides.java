package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
* HorizontalSlides.java Summary
* HorizontalSlides - constructor
* - move (pos) - move to pos with preset offset
* - move (pos, offset) - move to pos with custom offset
* - intakeOn() - turns intake on
* - intakeOff() - turns intake off
* - setIntakeSpeed() - set the intake speed 0-1
* */

/*
* names on config
* intHinge_l (expansion hub 4): left intake hinge
* intHinge_r (expansion hub 2): right intake hinge
* intake (expansion hub 3): intake cont. rotation servo
* */

public class HorizontalSlides extends SubsystemBase {
	private final Servo rightSlideServo;
	private final Servo leftSlideServo;
	private final CRServo intakeServo;
	private final Servo rightIntHinge;
	private final Servo leftIntHinge;
	double rightOffset = -0.005;
	double intakeOffset = 0.0;
	double intakeSpeed = 0.5;

	//preset positions
	double armUp;
	double armForward;
	double armBack;

	//0.63 max
	//0.5 min, intaking pos

	public HorizontalSlides (final HardwareMap hMap, final String rServoName, final String lServoName, final String intakeName, final String rightIntHingeName, final String leftIntHingeName) {
		rightSlideServo = hMap.get(Servo.class, rServoName);
		leftSlideServo = hMap.get(Servo.class, lServoName);
		intakeServo = hMap.get(CRServo.class, intakeName);
		rightIntHinge = hMap.get(Servo.class, rightIntHingeName);
		leftIntHinge = hMap.get(Servo.class, leftIntHingeName);
	}

	public void extend() {
		move(0.63);
	}

	public void contract() {
		move(0.5);
	}

	public void move(double pos) {	//move to position with preset offset
		rightSlideServo.setPosition(pos + rightOffset);
		leftSlideServo.setPosition(1-pos);
	}

	public void move(double pos, double offset) {	//move to position with offset you can choose (for testing with @Config)
		rightSlideServo.setPosition(pos + offset);
		leftSlideServo.setPosition(1-pos);
	}

	public void intakeOn() {	// make intake spin
		intakeServo.setPower(intakeSpeed);
	}

	public void intakeOff() {
		intakeServo.setPower(0);
	}

	public void setIntakeSpeed(double speed) {		//change intake spin power
		intakeSpeed = speed;
	}

	public void setIntHingePos(double pos) {
		rightIntHinge.setPosition(pos + intakeOffset);
		leftIntHinge.setPosition(1-pos);
	}

	public void setIntHingePos(double pos, double offset) {
		rightIntHinge.setPosition(pos + offset);
		leftIntHinge.setPosition(1-pos);
	}
}
