package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.commands.*;

public class FullTransfer extends SequentialCommandGroup {

	public FullTransfer(HorizontalSlides hSlides, Arm arm, VerticalSlides vSlides) {
		addCommands(
				new IntakeToArmTransfer(arm, hSlides, vSlides),
				new VerticalSlidesExtend(arm, vSlides)
		);
		addRequirements(hSlides, arm, vSlides);
	}
}
