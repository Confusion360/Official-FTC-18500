package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class VerticalSlides extends SubsystemBase{
	private final DcMotor motor;
	Gamepad gamepad;

	int SLIDE_MIN_POSITION = 0;
	int SLIDE_MAX_POSITION = 2300;	//calibrated on 22/10/24

	public VerticalSlides (final HardwareMap hMap, final String name, final Gamepad gamepad) {
		motor = hMap.get(DcMotor.class, name);
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		this.gamepad = gamepad;
	}

	public void extend () {		//extends arm to max pos
		motor.setTargetPosition(SLIDE_MAX_POSITION);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setPower(0.5);
	}

	public void reduce () {		//reduces arm to min pos
		motor.setTargetPosition(0);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setPower(0.5);
	}

	public void moveToPos (int pos) {	//move to pos from SLIDE_MIN_POS to SLIDE_MAX_POS
		motor.setTargetPosition(pos);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setPower(1);
	}

	public void slowUp () {
		motor.setPower(gamepad.left_stick_y * 0.1);
	}

	public double getTargetPos () {		//for telemetry
		return motor.getTargetPosition();
	}

	public double getCurrentPos () {	//for telemetry
		return motor.getCurrentPosition();
	}

	public void showPos(Telemetry telemetry) {
		// Show the current and target positions of the vertical slides on telemetry
		telemetry.addData("Vert Slides Encoder Position", motor.getCurrentPosition());
		telemetry.addData("Vert SLides Desired Position", motor.getTargetPosition());

		telemetry.update();
	}
}
