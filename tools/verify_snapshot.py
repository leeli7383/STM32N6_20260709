"""Verify original snapshot bytes and Intel HEX structure; does not run firmware."""

import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def verify_hex(path):
    upper = 0
    minimum = None
    maximum = None
    payload_size = 0
    eof = False
    records = 0
    expected_sizes = {1: 0, 2: 2, 3: 4, 4: 2, 5: 4}
    for number, line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        if not line:
            continue
        if eof:
            raise ValueError(f"{path.name}:{number}: record after EOF")
        if not line.startswith(":"):
            raise ValueError(f"{path.name}:{number}: missing record marker")
        record = bytes.fromhex(line[1:])
        if len(record) < 5 or len(record) != record[0] + 5:
            raise ValueError(f"{path.name}:{number}: invalid record length")
        if sum(record) & 255:
            raise ValueError(f"{path.name}:{number}: invalid checksum")
        count = record[0]
        address = int.from_bytes(record[1:3], "big")
        kind = record[3]
        data = record[4:-1]
        if kind == 0:
            if count:
                start = upper + address
                minimum = start if minimum is None else min(minimum, start)
                maximum = start + count - 1 if maximum is None else max(maximum, start + count - 1)
                payload_size += count
        elif kind in expected_sizes:
            if count != expected_sizes[kind] or address != 0:
                raise ValueError(f"{path.name}:{number}: invalid control record")
            if kind == 1:
                eof = True
            elif kind == 2:
                upper = int.from_bytes(data, "big") << 4
            elif kind == 4:
                upper = int.from_bytes(data, "big") << 16
        else:
            raise ValueError(f"{path.name}:{number}: unsupported record type {kind}")
        records += 1
    if not eof or minimum is None:
        raise ValueError(f"{path.name}: missing EOF or data")
    print(f"HEX OK {path.name}: {records} records, {payload_size} data bytes, "
          f"0x{minimum:08X}..0x{maximum:08X}")


def main():
    manifest = json.loads((ROOT / "docs/snapshot-manifest.json").read_text(encoding="utf-8"))
    entries = manifest["files"]
    expected = {item["path"] for item in entries}
    actual = {p.relative_to(ROOT).as_posix() for p in (ROOT / "test").rglob("*") if p.is_file()}
    if expected != actual or len(expected) != len(entries):
        raise ValueError(f"Snapshot inventory mismatch: missing={sorted(expected-actual)}, "
                         f"extra={sorted(actual-expected)}")
    total = 0
    for item in entries:
        path = ROOT / item["path"]
        if not path.resolve().is_relative_to((ROOT / "test").resolve()):
            raise ValueError("Manifest path outside test directory")
        data = path.read_bytes()
        if len(data) != item["bytes"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise ValueError(f"Snapshot bytes changed: {item['path']}")
        total += len(data)
    print(f"SNAPSHOT OK: {len(entries)} files, {total} bytes, SHA-256 matched")
    for path in sorted((ROOT / "test/Binary").glob("*.hex")):
        verify_hex(path)
    report = ROOT / "docs/design-report.pdf"
    if not report.read_bytes().startswith(b"%PDF-"):
        raise ValueError("Missing or invalid design report PDF")
    print("REPORT OK: PDF header present (not a semantic validation)")
    print("PASS: snapshot integrity and HEX format only; firmware build/hardware not tested")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
