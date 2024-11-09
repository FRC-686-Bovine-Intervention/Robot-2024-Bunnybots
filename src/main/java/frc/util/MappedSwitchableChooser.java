package frc.util;

import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;

/** A string chooser for the dashboard where the options can be changed on-the-fly. */
public class MappedSwitchableChooser<T> implements LoggedDashboardInput, Supplier<T> {
  private Map<String, T> options;
  private T selectedOption;
  private T activeOption;
  private T defaultOption;

  private final StringPublisher namePublisher;
  private final StringPublisher typePublisher;
  private final StringArrayPublisher optionsPublisher;
  private final StringPublisher defaultPublisher;
  private final StringPublisher activePublisher;
  private final StringPublisher selectedPublisher;
  private final LoggedDashboardString selectedInput;

  public MappedSwitchableChooser(String name, Map<String, T> options, T defaultOption) {
    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(name);
    namePublisher = table.getStringTopic(".name").publish();
    typePublisher = table.getStringTopic(".type").publish();
    optionsPublisher = table.getStringArrayTopic("options").publish();
    defaultPublisher = table.getStringTopic("default").publish();
    activePublisher = table.getStringTopic("active").publish();
    selectedPublisher = table.getStringTopic("selected").publish();
    selectedInput = new LoggedDashboardString(name + "/selected");
    Logger.registerDashboardInput(this);

    namePublisher.set(name);
    typePublisher.set("String Chooser");
    setOptions(options);
    setDefault(defaultOption);
    setActive(defaultOption);
    setSelected(defaultOption);
  }

  @Override
  public void periodic() {
    var selected = selectedInput.get();
    selectedOption = options.get(selected);
  }

  /** Updates the set of available options. */
  public void setOptions(Map<String, T> options) {
    if(options.isEmpty()) throw new UnsupportedOperationException("Options should not be empty. Try using Optionals or null to clear options");
    if(options.equals(this.options)) return;
    this.options = options;
    optionsPublisher.set(this.options.keySet().toArray(String[]::new));
  }

  public void setSelected(T selectedValue) {
    if (selectedValue.equals(this.selectedOption)) return;
    this.selectedOption = selectedValue;
    selectedPublisher.set(this.getKey(this.selectedOption));
  }

  public void setActive(T activeValue) {
    if (activeValue.equals(this.activeOption)) return;
    this.activeOption = activeValue;
    activePublisher.set(this.getKey(this.activeOption));
  }

  public void setDefault(T defaultValue) {
    if (defaultValue.equals(this.defaultOption)) return;
    this.defaultOption = defaultValue;
    defaultPublisher.set(this.getKey(this.defaultOption));
  }

  public Map<String, T> getOptions() {
    return options;
  }

  /** Returns the selected option. */
  @Override
  public T get() {
    return selectedOption;
  }

  public T getActive() {
    return activeOption;
  }

  public T getDefault() {
    return defaultOption;
  }

  private String getKey(T value) {
    return this.options.entrySet().stream().filter((e) -> e.getValue().equals(value)).map(Entry::getKey).findAny().orElse(null);
  }
}
