"""Create an installation key and signed ESP-NOW OTA packages (cryptography required).

Keep the private key outside version control and back it up securely. Station
and sensor base images must be built with the SAME generated public header.
"""
import argparse
import hashlib
from pathlib import Path
import struct
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec

ROOT = Path(__file__).resolve().parent
HEADER = struct.Struct("<IHBBII24s32sH72s14s")
IDENTITY = struct.Struct("<8sIBB24s")


def initialize(key_path, public_header):
    if key_path.exists() or public_header.exists():
        raise ValueError("Key/header already exists; refusing to replace the installation identity")
    key = ec.generate_private_key(ec.SECP256R1())
    key_path.parent.mkdir(parents=True, exist_ok=True)
    key_path.write_bytes(key.private_bytes(serialization.Encoding.PEM,
        serialization.PrivateFormat.PKCS8, serialization.NoEncryption()))
    public = key.public_key().public_bytes(serialization.Encoding.PEM,
        serialization.PublicFormat.SubjectPublicKeyInfo).decode("ascii")
    public_header.parent.mkdir(parents=True, exist_ok=True)
    public_header.write_text('#pragma once\nstatic constexpr char kOtaPublicKey[] = R"KEY(' +
                             public + ')KEY";\n', encoding="ascii")


def package(image, key):
    if not 256 < len(image) <= 0x1E0000 or image[0] != 0xE9:
        raise ValueError("Expected a sensor application firmware.bin within the OTA slot limit")
    positions = [i for i in range(len(image)) if image.startswith(b"WCHFW01\0", i)]
    if len(positions) != 1:
        raise ValueError("Expected exactly one embedded W-Charger firmware identity")
    _, release, pcb, protocol, version = IDENTITY.unpack_from(image, positions[0])
    if pcb not in (3, 4) or protocol != 5 or not release or not version[0] or b"\0" not in version:
        raise ValueError("Invalid embedded target/version")
    signed = struct.pack("<IHBBII24s32s", 0x3141544F, 1, pcb, protocol,
                         len(image), release, version, hashlib.sha256(image).digest())
    signature = key.sign(signed, ec.ECDSA(hashes.SHA256()))
    if len(signature) > 72:
        raise ValueError("Unexpected signature size")
    return signed + struct.pack("<H72s14s", len(signature), signature, bytes(14)) + image


def verify(data, public_key):
    if len(data) < HEADER.size:
        raise ValueError("Truncated package")
    magic, fmt, pcb, protocol, size, release, version, digest, n, signature, reserved = HEADER.unpack_from(data)
    if magic != 0x3141544F or fmt != 1 or pcb not in (3, 4) or protocol != 5 or not 0 < n <= 72:
        raise ValueError("Invalid manifest")
    image = data[HEADER.size:]
    if len(image) != size or hashlib.sha256(image).digest() != digest:
        raise ValueError("Image size/hash mismatch")
    public_key.verify(signature[:n], data[:72], ec.ECDSA(hashes.SHA256()))
    return pcb, release, version.split(b"\0")[0].decode("ascii")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["init", "pack", "verify"])
    parser.add_argument("--key", type=Path, default=ROOT.parent / ".tools/ota/signing.key")
    parser.add_argument("--public-header", type=Path, default=ROOT / "shared/include/ota_public_key.h")
    parser.add_argument("--image", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.command == "init":
        initialize(args.key, args.public_header)
        print("Installation signing key and public header created. Back up the private key.")
        return
    key = serialization.load_pem_private_key(args.key.read_bytes(), password=None)
    if not isinstance(key, ec.EllipticCurvePrivateKey) or not isinstance(key.curve, ec.SECP256R1):
        parser.error("Expected a P-256 installation key")
    if args.image is None:
        parser.error("--image is required")
    data = args.image.read_bytes()
    if args.command == "pack":
        if args.output is None:
            parser.error("--output is required")
        data = package(data, key)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_bytes(data)
    print("Verified PCB V%d, release %d, version %s" % verify(data, key.public_key()))


if __name__ == "__main__":
    main()
