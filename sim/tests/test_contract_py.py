"""Python contract mirror against the shared test vectors (same bytes as the C tests)."""
import json
import random
from pathlib import Path

from m3sim import contract as ct
from m3sim.plant import Plant, PlantParams

VECTORS = json.loads((Path(__file__).resolve().parents[2] / "contract" / "test_vectors.json").read_text())


def test_crc_check_value():
    assert ct.crc16(VECTORS["crc_check"]["ascii"].encode()) == int(VECTORS["crc_check"]["crc"], 16)


def test_vectors_encode_and_decode():
    for v in VECTORS["frames"]:
        frame = ct.encode_frame(v["type"], v["session"], v["seq"], ct.pack(v["type"], *v["fields"]))
        assert frame.hex() == v["hex"], v["name"]
        (decoded,) = ct.Decoder().feed(bytes.fromhex(v["hex"]))
        assert ct.unpack(decoded.type, decoded.payload) == tuple(v["fields"])
        assert (decoded.session, decoded.seq) == (v["session"], v["seq"])


def test_corrupted_frames_rejected():
    good = bytes.fromhex(VECTORS["frames"][2]["hex"])
    dec = ct.Decoder()
    rng = random.Random(1)
    accepted = 0
    for _ in range(1000):
        bad = bytearray(good)
        bad[rng.randrange(4, len(bad))] ^= 1 << rng.randrange(8)
        accepted += len(dec.feed(bytes(bad)))
    assert accepted == 0
    assert dec.crc_errors == 1000


def test_seq_tracker():
    t = ct.SeqTracker()
    assert t.accept(5, 10) == "new_session"
    assert t.accept(5, 11) == "new"
    assert t.accept(5, 11) == "old"
    assert t.accept(6, 0) == "new_session"


def test_plant_hover_and_rest():
    p = PlantParams()
    assert abs(p.hover_rpm() - 3000.0) < 1.0
    plant = Plant(p)
    for _ in range(1000):
        plant.step(0.0, 0.001)
    assert plant.h_mm == 0.0  # rests on the grille without air
