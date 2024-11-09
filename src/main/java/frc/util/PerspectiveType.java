package frc.util;

import java.util.LinkedHashMap;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import frc.util.Alert.AlertType;

public enum PerspectiveType {
	PosX(
		MatBuilder.fill(Nat.N2(), Nat.N2(),
			+0,+1,
			-1,+0
		)
	),
	NegX(
		MatBuilder.fill(Nat.N2(), Nat.N2(),
			+0,-1,
			+1,+0
		)
	),
	PosY(
		MatBuilder.fill(Nat.N2(), Nat.N2(),
			+1,+0,
			+0,+1
		)
	),
	NegY(
		MatBuilder.fill(Nat.N2(), Nat.N2(),
			-1,+0,
			+0,-1
		)
	),
	;
	private final Matrix<N2, N2> spectatorToField;
	PerspectiveType(Matrix<N2, N2> spectatorToField) {
		this.spectatorToField = spectatorToField;
	}
	public Vector<N2> toField(Vector<N2> vec) {
		return new Vector<N2>(spectatorToField.times(vec));
	}

	private static final MappedSwitchableChooser<PerspectiveType> chooser;
	static {
		var map = new LinkedHashMap<String, PerspectiveType>();
		map.put("Blue Left (+Y)", PosY);
		map.put("Blue Right (-Y)", NegY);
		map.put("Blue Alliance (+X)", PosX);
		map.put("Red Alliance (-X)", NegX);
		chooser = new MappedSwitchableChooser<>(
			"Perspective Type",
			map,
			PosY
		);
	}
	
    static {
        Logger.registerDashboardInput(new LoggedDashboardInput() {
			private static final SuppliedEdgeDetector FMS_edge_detector = new SuppliedEdgeDetector(DriverStation::isFMSAttached);
			private static final Alert comp_wrong_perspective_alert = new Alert("Competition Environment detected, but selected Perspective does not match the Alliance", AlertType.WARNING);
            public void periodic() {
                FMS_edge_detector.update();
				if(FMS_edge_detector.risingEdge()) {
					chooser.setSelected(getAllianceType());
				}
				chooser.setActive(chooser.get());
				comp_wrong_perspective_alert.set(Environment.isCompetition() && getCurrentType() != getAllianceType());
            }
        });
    }

	public static PerspectiveType getAllianceType() {
		return AllianceFlipUtil.shouldFlip() ? NegX : PosX;
	}

	public static PerspectiveType getCurrentType() {
		return chooser.getActive();
	}
}
