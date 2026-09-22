import unittest

from navigation.lidar_protocol import decode_lidar_packet, parse_lidar_message


class TestLidarMessageParsing(unittest.TestCase):
    def test_scan_packet(self):
        self.assertEqual(
            parse_lidar_message("LIDAR_SCAN:0,10;90,20;-90,5"),
            [(0.0, 10.0), (90.0, 20.0), (-90.0, 5.0)],
        )

    def test_json_packet(self):
        self.assertEqual(
            parse_lidar_message('{"scan":[{"angle":45,"distance_cm":12}]}'),
            [(45.0, 12.0)],
        )

    def test_invalid_packet_is_ignored(self):
        self.assertEqual(parse_lidar_message("COMPLETED:MOVE:10.0"), [])
        self.assertEqual(parse_lidar_message("LIDAR_SCAN:10,-1;bad,20"), [])

    def test_firmware_packet_decoding(self):
        self.assertEqual(decode_lidar_packet("3D01009001"), (0.0, 10.0))
        self.assertIsNone(decode_lidar_packet("COMPLETED"))


if __name__ == "__main__":
    unittest.main()