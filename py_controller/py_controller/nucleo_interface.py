import struct
import threading

import serial
import structlog
from serial.tools import list_ports

# Configure logging and structlog


class NucleoController:
    """
    A class to communicate with a Nucleo board over a serial connection.
    Automatically detects the Nucleo's COM port and provides methods to send commands.
    """

    SERVO_TIMINGS: dict[int, tuple[int, int]] = {
        0: (100, 470),
    }

    def __init__(self, baud_rate: int = 115_200, timeout: float = 0.01):
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_port = None
        self.lock = threading.Lock()
        self.logger = structlog.get_logger()
        self.connect()

    def connect(self) -> None:
        """
        Attempts to find the Nucleo board's COM port and establish a serial connection.
        """
        port_name = self.find_nucleo_port()
        if port_name:
            self.serial_port = serial.Serial(port_name, self.baud_rate, timeout=self.timeout)
            self.logger.info("Connected to Nucleo board", port=port_name)
        else:
            raise ConnectionError("Could not find Nucleo board. Please ensure it is connected.")

    @staticmethod
    def find_nucleo_port():
        """
        Scans available COM ports to find the one connected to the Nucleo board.
        """
        ports = list_ports.comports()
        for port in ports:
            if (
                "STM" in port.description
                or "Nucleo" in port.description
                or (port.manufacturer and "STMicroelectronics" in port.manufacturer)
            ):
                return port.device
        return None

    @staticmethod
    def map_range(
        value: float | int,
        input_min: float | int,
        input_max: float | int,
        output_min: float | int,
        output_max: float | int,
    ) -> float:
        """
        Maps a value from one range to another and clamps it within the output range.

        Parameters:
            value (float): The input value to be mapped.
            input_min (float): The minimum of the input range.
            input_max (float): The maximum of the input range.
            output_min (float): The minimum of the output range.
            output_max (float): The maximum of the output range.

        Returns:
            float: The mapped and clamped value in the output range.
        """
        # Calculate the proportion of 'value' within the input range
        proportion = (value - input_min) / (input_max - input_min)

        # Map the proportion to the output range
        mapped_value = output_min + proportion * (output_max - output_min)

        # Clamp the mapped value to the output range
        clamped_value = max(min(mapped_value, output_max), output_min)

        return clamped_value

    @staticmethod
    def configure_structlog_as_recommended():
        structlog.configure(
            processors=[
                structlog.processors.add_log_level,
                structlog.processors.TimeStamper(fmt="iso"),
                structlog.processors.StackInfoRenderer(),
                structlog.dev.ConsoleRenderer(),
            ],
            cache_logger_on_first_use=True,
        )

    def close(self) -> None:
        """
        Stops all motors and closes the serial connection.
        """
        if self.serial_port and self.serial_port.is_open:
            # Stop all motors before closing
            self.stop_all_steppers()
            self.serial_port.close()
            self.logger.info("Serial connection closed.")

    def send_command(self, command_byte: str, data_bytes: bytes = b"") -> None:
        """
        Sends a command to the Nucleo board.
        """
        assert self.serial_port is not None, "Must initialize serial!"
        assert len(command_byte) == 1, "Command must be a single character."
        with self.lock:
            header = b"M"
            command = command_byte.encode("ascii")
            self.serial_port.write(header + command + data_bytes)

    def receive_data(self, num_bytes: int) -> bytes:
        """
        Receives a specified number of bytes from the Nucleo board.
        """
        assert self.serial_port is not None, "Must initialize serial!"
        with self.lock:
            data = self.serial_port.read(num_bytes)
        return data

    def set_servo_pwm(self, channel: int, on_time: int, off_time: int) -> None:
        """
        Sends a command to set a servo position.
        """
        if not (0 <= channel <= 15):
            raise ValueError("Channel must be between 0 and 15.")
        if not (0 <= on_time <= 4095) or not (0 <= off_time <= 4095):
            raise ValueError("On time and off time must be between 0 and 4095.")
        data = struct.pack("<BHH", channel, on_time, off_time)
        self.send_command("s", data)
        self.logger.info("Set servo", channel=channel, on_time=on_time, off_time=off_time)

    def set_servo_angle(self, channel: int, angle: float) -> None:
        if not (timing := self.SERVO_TIMINGS.get(channel)):
            raise ValueError(f"Unknown timings for channel {channel}!")
        off_time = self.map_range(angle, 0, 360, *timing)
        return self.set_servo_pwm(channel, 0, round(off_time))

    def read_angles(self) -> tuple[int, int, int]:
        """
        Sends a command to read angles from the Nucleo board.
        Returns a tuple of three angles.
        """
        self.send_command("a")
        data = self.receive_data(12)
        if len(data) != 12:
            self.logger.error("Failed to receive angle data.")
            raise IOError("Failed to receive angle data.")
        angles = struct.unpack("<iii", data)
        self.logger.info("Received angles", angles=angles)
        return tuple(angle / 4096 * 360 for angle in angles)

    def set_wheel_speeds(self, speeds: tuple[int, int, int]) -> None:
        """
        Sends a command to set wheel speeds.
        """
        data = struct.pack("<iii", *speeds)
        self.send_command("u", data)
        self.logger.info("Set wheel speeds", speeds=speeds)

    def stop_all_steppers(self) -> None:
        """
        Sends a command to stop all stepper motors.
        """
        self.send_command("x")
        self.logger.info("Sent command to stop all steppers.")

    def ping(self) -> bool:
        """
        Sends a ping command and waits for a 'pong' response.
        Returns True if 'pong' is received, False otherwise.
        """
        self.send_command("p")
        data = self.receive_data(4)
        if data == b"pong":
            self.logger.info("Received pong response.")
            return True
        else:
            self.logger.warning("No pong response received.")
            return False

    def set_inverse_kinematics(self, x_dot: int, y_dot: int, theta_dot: int) -> None:
        """
        Sends a command to set wheel velocities via inverse kinematics.
        """
        # Assuming x_dot, y_dot, theta_dot are integers
        data = struct.pack("<iii", x_dot, y_dot, theta_dot)
        self.send_command("k", data)
        self.logger.info("Set inverse kinematics", x_dot=x_dot, y_dot=y_dot, theta_dot=theta_dot)

    def __enter__(self):
        """
        Allows use of the class as a context manager.
        """
        return self

    def __exit__(self, *_) -> None:
        """
        Ensures the serial connection is closed when exiting a context.
        """
        self.close()


class DummyNucleoController:
    """
    Dummy NucleoController class for testing purposes.
    Logs actions instead of communicating with hardware.
    """

    SERVO_TIMINGS = {
        0: (100, 470),
        1: (100, 470),
    }

    def __init__(self, baud_rate: int = 115200, timeout: float = 1):
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.lock = None  # Not needed in dummy
        self.logger = structlog.get_logger()
        self.logger.info(
            "Initialized dummy NucleoController", baud_rate=self.baud_rate, timeout=self.timeout
        )

    @staticmethod
    def configure_structlog_as_recommended():
        structlog.configure(
            processors=[
                structlog.processors.add_log_level,
                structlog.processors.TimeStamper(fmt="iso"),
                structlog.processors.StackInfoRenderer(),
                structlog.dev.ConsoleRenderer(),
            ],
            cache_logger_on_first_use=True,
        )

    def connect(self) -> None:
        self.logger.info("Dummy connect called")

    def close(self) -> None:
        self.logger.info("Dummy close called")

    def send_command(self, command_byte: str, data_bytes: bytes = b"") -> None:
        self.logger.info("Dummy send_command", command_byte=command_byte, data_bytes=data_bytes)

    def receive_data(self, num_bytes: int) -> bytes:
        self.logger.info("Dummy receive_data", num_bytes=num_bytes)
        # Return dummy data
        return b"\x00" * num_bytes

    def set_servo_pwm(self, channel: int, on_time: int, off_time: int) -> None:
        self.logger.info("Dummy set_servo_pwm", channel=channel, on_time=on_time, off_time=off_time)

    def set_servo_angle(self, channel: int, angle: float) -> None:
        if not (timing := self.SERVO_TIMINGS.get(channel)):
            raise ValueError(f"Unknown timings for channel {channel}!")
        off_time = self.map_range(angle, 0, 360, *timing)
        self.logger.info("Dummy set_servo_angle", channel=channel, angle=angle, off_time=off_time)

    def read_angles(self) -> tuple[float, float, float]:
        self.logger.info("Dummy read_angles called")
        # Return dummy angles
        return (0.0, 0.0, 0.0)

    def set_wheel_speeds(self, speeds: tuple[int, int, int]) -> None:
        self.logger.info("Dummy set_wheel_speeds", speeds=speeds)

    def stop_all_steppers(self) -> None:
        self.logger.info("Dummy stop_all_steppers called")

    def ping(self) -> bool:
        self.logger.info("Dummy ping called")
        return True  # Always successful in dummy

    def set_inverse_kinematics(self, x_dot: int, y_dot: int, theta_dot: int) -> None:
        self.logger.info(
            "Dummy set_inverse_kinematics", x_dot=x_dot, y_dot=y_dot, theta_dot=theta_dot
        )

    @staticmethod
    def map_range(
        value: float | int,
        input_min: float | int,
        input_max: float | int,
        output_min: float | int,
        output_max: float | int,
    ) -> float:
        # Same as before
        proportion = (value - input_min) / (input_max - input_min)
        mapped_value = output_min + proportion * (output_max - output_min)
        clamped_value = max(min(mapped_value, output_max), output_min)
        return clamped_value

    def __enter__(self):
        self.logger.info("Dummy __enter__ called")
        return self

    def __exit__(self, *_) -> None:
        self.logger.info("Dummy __exit__ called")
        self.close()
