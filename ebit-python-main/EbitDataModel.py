import csv
from dataclasses import dataclass, fields


@dataclass
class EbitComponent:
    """Class for each ebit component, purposefully matched the variable names to csv name field."""

    # Fields read in from the CSV. If a field isn't relevant for a component, it will be set to a value of None.
    name: str
    output_card: str
    output_pin: str
    output_min_volts: float
    output_max_volts: float
    output_default_value: float
    output_voltage_slope: float
    output_voltage_y_intercept: float
    input_voltage_card: str
    input_voltage_pin: str
    voltage_slope: float
    voltage_y_intercept: float
    input_current_card: str
    input_current_pin: str
    current_slope: float
    current_y_intercept: float
    power_threshold: float

    # These are values not set in the CSV file.
    output_voltage_value: float = None
    input_voltage_value: float = None
    input_current_value: float = None

    def __post_init__(self):
        """
        The csv reader sets our attributes all as strings, even though we specify a type.
        This function uses the specified types to cast automatically when imported.
        """
        for field in fields(self):
            value = getattr(self, field.name)
            if value is None:
                continue
            elif value == '':
                setattr(self, field.name, None)
            else:
                setattr(self, field.name, field.type(value))


class EbitDataModel:
    components = {}

    def __init__(self):
        # Read io_config and populate components
        with open('io_config.csv') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                self.components[row["name"]] = EbitComponent(**row)


if __name__ == "__main__":
    ebit = EbitDataModel()
    print(ebit.components)
