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

/*
 * Arm.java Summary
 * ArmSystem - constructor
 * grab - closes claw
 * release - opens claw
 * moveArm - move arm to specified location
 * */

/*
 * Stuff on config
 * claw_hinge (expansion hub 1)
 * claw (expansion hub 5)
 * */

public class Arm extends SubsystemBase{

	private final Servo arm_l;
	private final Servo arm_r;
	private final Servo claw;
	private final Servo claw_hinge;

	//double armServoOffset = -0.04;	//fixing misaligned middles

	public Arm(final HardwareMap hMap, final String alName, final String arName, final String clawName, final String crName) {
		arm_l = hMap.get(Servo.class, alName);
		arm_r = hMap.get(Servo.class, arName);
		claw = hMap.get(Servo.class, clawName);
		claw_hinge = hMap.get(Servo.class, crName);
	}

	public void grab(double pos) {
		claw.setPosition(pos);
	}

	public void release() {
		claw.setPosition(0.5);
	}

	public void moveHinge(double pos) {
		claw_hinge.setPosition(pos);
		//middle - 0.48
		//max - 0.75
		//min - 0.21
	}

	public void moveArm(double pos) {
		arm_l.setPosition(1-pos);
		arm_r.setPosition(pos);
	}
}
