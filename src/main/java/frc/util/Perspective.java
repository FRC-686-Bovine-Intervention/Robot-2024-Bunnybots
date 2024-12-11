package frc.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.LinkedHashMap;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import edu.wpi.first.wpilibj.DriverStation;

public class Perspective {
	private Matrix<N2,N2> spectatorToField;
	private Perspective(Matrix<N2,N2> spectatorToField) {
		this.spectatorToField = spectatorToField;
	}
	
	public Vector<N2> toField(Vector<N2> vec) {
		return new Vector<N2>(spectatorToField.times(vec));
	}
	
	private static final Matrix<N2,N2> joystickToRobot = MathExtraUtil.rotationMatrix(Rotation2d.kCW_90deg);
	private static final Perspective posY = new Perspective(MathExtraUtil.rotationMatrix(Rotation2d.kCCW_90deg).times(joystickToRobot));
	private static final Perspective negY = new Perspective(MathExtraUtil.rotationMatrix(Rotation2d.kCW_90deg).times(joystickToRobot));
	private static final Perspective posX = new Perspective(MathExtraUtil.rotationMatrix(Rotation2d.kZero).times(joystickToRobot));
	private static final Perspective negX = new Perspective(MathExtraUtil.rotationMatrix(Rotation2d.k180deg).times(joystickToRobot));
	private static final LoggedTunableMeasure<AngleUnit> customTunable = new LoggedTunableMeasure<>("Perspective/Custom", Degrees.zero());
	private static final Perspective custom = new Perspective(MathExtraUtil.rotationMatrix(Rotation2d.fromRadians(customTunable.in(Radians))).times(joystickToRobot));

	private static final MappedSwitchableChooser<Perspective> chooser;
	static {
		var map = new LinkedHashMap<String, Perspective>();
		map.put("Blue Left (+Y)", posY);
		map.put("Blue Right (-Y)", negY);
		map.put("Blue Alliance (+X)", posX);
		map.put("Red Alliance (-X)", negX);
		map.put("Custom", custom);
		chooser = new MappedSwitchableChooser<>(
			"Perspective",
			map,
			posY
		);
	}
	
    static {
        Logger.registerDashboardInput(new LoggedDashboardInput() {
			private static final SuppliedEdgeDetector FMS_edge_detector = new SuppliedEdgeDetector(DriverStation::isFMSAttached);
			private static final Alert comp_wrong_perspective_alert = new Alert("Competition Environment detected, but selected Perspective does not match the Alliance", AlertType.kWarning);
            public void periodic() {
                FMS_edge_detector.update();
				if (FMS_edge_detector.risingEdge()) {
					chooser.setSelected(getAlliance());
				}
				if (chooser.get() == custom) {
					custom.spectatorToField = MathExtraUtil.rotationMatrix(Rotation2d.fromRadians(customTunable.in(Radians))).times(joystickToRobot);
				}
				chooser.setActive(chooser.get());
				comp_wrong_perspective_alert.set(Environment.isCompetition() && getCurrent() != getAlliance());
            }
        });
    }

	public static Perspective getAlliance() {
		return AllianceFlipUtil.shouldFlip() ? negX : posX;
	}

	public static Perspective getCurrent() {
		return chooser.getActive();
	}
}
