"""USART3 / 14-byte SDP protocol probe. Requires: python -m pip install pyserial.
Run only with FIRMWARE_HARDWARE_TEST=1, FIRMWARE_HW_TEST_KIND=3.
UART is 3.3-V TTL, 115200 8N1, through an external USB-UART adapter.
"""
import argparse
import time


def make_packet(servo=(114, 117, 108, 100, 0)):
    packet = bytearray((0xAA, 0x55, 0, 0, 0, 100, 100, 100, *servo, 0))
    assert len(packet) == 14
    packet[13] = sum(packet[2:13]) & 0xFF
    return packet


def main():
    parser = argparse.ArgumentParser(description="STM32 SDP UART hardware test")
    parser.add_argument("--port", required=True, help="COM4 or /dev/ttyUSB0")
    args = parser.parse_args()
    import serial
    with serial.Serial(args.port, baudrate=115200, bytesize=8,
                       parity="N", stopbits=1, timeout=1.0) as s:
        cases = [
            ("valid", make_packet(), 1),
            ("bad checksum", make_packet(), 0),
            ("bad servo byte", make_packet(servo=(114, 117, 108, 100, 42)), 0),
        ]
        cases[1][1][13] ^= 1
        for label, frame, expected in cases:
            s.reset_input_buffer()
            s.write(frame)
            reply = s.read(3)
            passed = (reply == bytes((0xA5, 0x5A, expected)))
            print(f"{label}: {'PASS' if passed else 'FAIL'}; reply={reply.hex(' ')}")
            time.sleep(0.25)


if __name__ == "__main__":
    main()
