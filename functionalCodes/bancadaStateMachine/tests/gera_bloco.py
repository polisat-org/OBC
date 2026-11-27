"""Gera um bloco de 4080 B no formato do HERMES Simples (20 x Plane_t de 204 B, '<8s8s45fII').

3 aviões válidos (um sem posição), 15 slots vazios e 2 registros corrompidos (ICAO não inteiro e
latitude impossível), para testar a recepção e a validação do OBC (payload_downlink.c).
"""
import math
import struct
import sys

FMT = struct.Struct("<8s8s45fII")
NAN = math.nan
# índices dos floats a partir de icaoID (ver hermes_protocol.h)
I_ICAO, I_MET, I_ALT, I_GS, I_TRACK, I_LAT, I_LON, I_SEEN_POS = 0, 1, 3, 5, 9, 21, 22, 25


def plane(icao=0.0, met=NAN, lat=NAN, lon=NAN, alt=NAN, gs=NAN, track=NAN, seen_pos=NAN, empty=False, flight=b""):
    f = [0.0 if empty else NAN] * 45
    for i, v in ((I_ICAO, icao), (I_MET, met), (I_LAT, lat), (I_LON, lon), (I_ALT, alt), (I_GS, gs), (I_TRACK, track), (I_SEEN_POS, seen_pos)):
        if not empty:
            f[i] = v
    return FMT.pack(flight, b"", *f, 0, 0)


slots = [
    plane(0xE48C01, 12.5, -23.55, -46.63, 35000, 450.5, 90.0, 1.5, flight=b"TAM3054 "),
    plane(0x4B1817, 20.0, 10.0, 170.25, 1000, 120, 359.9, 0.0, flight=b"GLO12\x01XY"),   # byte invalido corta
    plane(0xABCDEF, 21.0, alt=15000, gs=300),
] + [plane(empty=True)] * 14 + [
    plane(1234.5, 1.0, 0.0, 0.0),          # ICAO não inteiro
    plane(0x123456, 1.0, 123.0, 0.0),      # latitude impossível
    plane(empty=True),
]
bloco = b"".join(slots)
assert len(bloco) == 4080
open(sys.argv[1], "wb").write(bloco)
