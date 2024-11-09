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

	private double firstPos = 0.49;	  //angle 19.962
	private double secondPos = 0.536;  // angle 31.88
	private double thirdPos = 0.59;	  //angle 45.7
	private double fourthPos = 0.66;  // angle 64.47
	private double[] slidePositions = {firstPos, secondPos, thirdPos, fourthPos};
	private int posIndex = 0;
	private boolean out = true;



	// 0.40 - entry
	// 0.45 - 1st hover
	// 0.48 - 2nd hover
	// 0.39 - down
	private double[] hingePositions = {0.40, 0.46, 0.49};
	private int hingeIndex = 0;

	//0.66 max
	//0.49 min, intaking pos

	//0.45 hover pos inside submersible

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
		hingeIndex = 0;
		rightSlideServo.setPosition(pos + rightOffset);
		leftSlideServo.setPosition(1-pos);
	}

	public void cycleSlidePos() {
		if (out) {
			move(slidePositions[posIndex]);
			posIndex++;
		} else {
			move(slidePositions[posIndex]);
			posIndex--;
		}
		  if (posIndex == 3 || posIndex == 0) {
			out = !out;
		}
	}

	public void cycleHingePos(Arm arm) {
		if (hingeIndex == 3) hingeIndex = 0;
		setHingePos(hingePositions[hingeIndex]);
		if (hingeIndex == 2) {
			intakeOn();
		}
		hingeIndex++;
	}

	public void intakeOn() {	// make intake spin
		intakeServo.setPower(-1);
	}

	public void intakeOff() { // make intake turn off
		intakeServo.setPower(0);
	}

	public void intakeEject(){intakeServo.setPower(1);} // make intake eject sample

	public void setHingePos(double pos) {
		hingeIndex = 0;
		rightIntHinge.setPosition(pos + 0.02);
		leftIntHinge.setPosition(pos);
	}

	private static void sleep(long milliseconds) {
		try {
			Thread.sleep(milliseconds);
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
		}
	}
}
