package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class ArmSystem extends SubsystemBase{

	private final Servo arm_l;
	private final Servo arm_r;
	private final Servo claw;
	private final Servo claw_rotate;

	double armServoOffset = -0.04;	//fixing misaligned middles

	public ArmSystem(final HardwareMap hMap, final String alName, final String arName, final String clawName, final String crName) {
		arm_l = hMap.get(Servo.class, alName);
		arm_r = hMap.get(Servo.class, arName);
		claw = hMap.get(Servo.class, clawName);
		claw_rotate = hMap.get(Servo.class, crName);
	}

	public void grab() {
		//claw closing code
	}

	public void release() {
		//claw closing code
	}



	public void moveArm(double pos) {
		arm_l.setPosition(pos + armServoOffset);
		arm_r.setPosition(1-pos);
	}
}
