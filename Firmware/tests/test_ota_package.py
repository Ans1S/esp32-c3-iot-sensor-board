"""Exercise actual package signing and verification, including malformed inputs."""
import importlib.util
import csv
import hashlib
from pathlib import Path
import struct
import tempfile
import unittest
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives import hashes

spec = importlib.util.spec_from_file_location("ota_package", Path(__file__).resolve().parents[1] / "ota_package.py")
ota = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ota)


class PackageTests(unittest.TestCase):
    def setUp(self):
        self.key = ec.generate_private_key(ec.SECP256R1())
        self.image = b"\xe9" + bytes(300) + ota.IDENTITY.pack(b"WCHFW01", 40001, 4, 5, b"4.0.1") + bytes(400)

    def test_roundtrip_and_hardware(self):
        self.assertEqual(ota.verify(ota.package(self.image, self.key), self.key.public_key()), (4, 40001, "4.0.1"))

    def test_every_header_and_image_byte_is_integrity_checked(self):
        package = ota.package(self.image, self.key)
        # Signed manifest fields, signature, and complete firmware are protected.
        n = struct.unpack_from("<H", package, 72)[0]
        for index in list(range(72)) + list(range(74, 74+n)) + list(range(160, len(package))):
            damaged = bytearray(package); damaged[index] ^= 1
            with self.assertRaises(Exception, msg=str(index)):
                ota.verify(damaged, self.key.public_key())

    def test_wrong_key_truncation_and_append(self):
        package = ota.package(self.image, self.key)
        with self.assertRaises(Exception):
            ota.verify(package, ec.generate_private_key(ec.SECP256R1()).public_key())
        for damaged in (package[:20], package[:-1], package + b"x"):
            with self.assertRaises(ValueError): ota.verify(damaged, self.key.public_key())

    def test_missing_duplicate_identity_and_oversize(self):
        for image in (bytes(1000), self.image * 2,
                      self.image + bytes(ota.SLOT_SIZE + 1 - len(self.image)),
                      b"\xe9" + bytes(300) + b"WCHFW01\0"):
            with self.assertRaises(ValueError): ota.package(image, self.key)

    def test_actual_partition_boundary_and_signed_oversize_rejection(self):
        partitions = Path(__file__).resolve().parents[1] / "sensor/partitions_recording.csv"
        rows = csv.reader(line for line in partitions.read_text().splitlines()
                          if line.strip() and not line.startswith("#"))
        app_sizes = [int(row[4].strip(), 0) for row in rows if row[1].strip() == "app"]
        self.assertEqual(app_sizes, [ota.SLOT_SIZE, ota.SLOT_SIZE])
        largest = self.image + bytes(ota.SLOT_SIZE - len(self.image))
        self.assertEqual(ota.verify(ota.package(largest, self.key), self.key.public_key())[0], 4)
        oversized = largest + b"\0"
        signed = struct.pack("<IHBBII24s32s", 0x3141544F, 1, 4, 5,
                             len(oversized), 40001, b"4.0.1", hashlib.sha256(oversized).digest())
        signature = self.key.sign(signed, ec.ECDSA(hashes.SHA256()))
        data = signed + struct.pack("<H72s14s", len(signature), signature, bytes(14)) + oversized
        with self.assertRaises(ValueError): ota.verify(data, self.key.public_key())

    def test_key_init_never_replaces_existing_identity(self):
        with tempfile.TemporaryDirectory() as folder:
            key, header = Path(folder) / "signing.key", Path(folder) / "public.h"
            ota.initialize(key, header)
            original = key.read_bytes()
            with self.assertRaises(ValueError): ota.initialize(key, header)
            self.assertEqual(key.read_bytes(), original)


if __name__ == "__main__": unittest.main()
