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
import edu.wpi.first.wpilibj.DriverStation;
import frc.util.Alert.AlertType;

public class PerspectiveType {
	private Matrix<N2,N2> spectatorToField;
	private PerspectiveType(Matrix<N2,N2> spectatorToField) {
		this.spectatorToField = spectatorToField;
	}
	
	public Vector<N2> toField(Vector<N2> vec) {
		return new Vector<N2>(spectatorToField.times(vec));
	}
	
	private static final Matrix<N2,N2> joystickToRobot = MathExtraUtil.rotationMatrix(Rotation2d.kCW_90deg);
	private static final PerspectiveType posY = new PerspectiveType(MathExtraUtil.rotationMatrix(Rotation2d.kCCW_90deg).times(joystickToRobot));
	private static final PerspectiveType negY = new PerspectiveType(MathExtraUtil.rotationMatrix(Rotation2d.kCW_90deg).times(joystickToRobot));
	private static final PerspectiveType posX = new PerspectiveType(MathExtraUtil.rotationMatrix(Rotation2d.kZero).times(joystickToRobot));
	private static final PerspectiveType negX = new PerspectiveType(MathExtraUtil.rotationMatrix(Rotation2d.k180deg).times(joystickToRobot));
	private static final LoggedTunableMeasure<AngleUnit> customTunable = new LoggedTunableMeasure<>("Perspective/Custom", Degrees.zero());
	private static final PerspectiveType custom = new PerspectiveType(MathExtraUtil.rotationMatrix(Rotation2d.fromRadians(customTunable.in(Radians))).times(joystickToRobot));

	private static final MappedSwitchableChooser<PerspectiveType> chooser;
	static {
		var map = new LinkedHashMap<String, PerspectiveType>();
		map.put("Blue Left (+Y)", posY);
		map.put("Blue Right (-Y)", negY);
		map.put("Blue Alliance (+X)", posX);
		map.put("Red Alliance (-X)", negX);
		map.put("Custom", custom);
		chooser = new MappedSwitchableChooser<>(
			"Perspective Type",
			map,
			posY
		);
	}
	
    static {
        Logger.registerDashboardInput(new LoggedDashboardInput() {
			private static final SuppliedEdgeDetector FMS_edge_detector = new SuppliedEdgeDetector(DriverStation::isFMSAttached);
			private static final Alert comp_wrong_perspective_alert = new Alert("Competition Environment detected, but selected Perspective does not match the Alliance", AlertType.WARNING);
            public void periodic() {
                FMS_edge_detector.update();
				if (FMS_edge_detector.risingEdge()) {
					chooser.setSelected(getAllianceType());
				}
				if (chooser.get() == custom) {
					custom.spectatorToField = MathExtraUtil.rotationMatrix(Rotation2d.fromRadians(customTunable.in(Radians))).times(joystickToRobot);
				}
				chooser.setActive(chooser.get());
				comp_wrong_perspective_alert.set(Environment.isCompetition() && getCurrentType() != getAllianceType());
            }
        });
    }

	public static PerspectiveType getAllianceType() {
		return AllianceFlipUtil.shouldFlip() ? negX : posX;
	}

	public static PerspectiveType getCurrentType() {
		return chooser.getActive();
	}
}
