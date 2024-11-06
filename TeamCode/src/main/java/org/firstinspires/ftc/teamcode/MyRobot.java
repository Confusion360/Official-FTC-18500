package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;
import com.arcrobotics.ftclib.command.button.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;



public class MyRobot extends Robot {


	public enum OpModeType {
		TELEOP, AUTO
	}

	public MyRobot(OpModeType type) {
		if (type == OpModeType.TELEOP) {
			initTele();
		} else {
			initAuto();
		}
	}

	public void initTele() {

	}

	public void initAuto() {

	}
}


