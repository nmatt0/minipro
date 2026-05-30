r"""
Derive minipro's infoic.xml `variant` field from a V13.19 InfoICT76.dll
0x74-byte chip descriptor.

REVERSE-ENGINEERED FROM Xgpro_T76.exe (V13.19), 2026-05-29.

    variant = (algo_number << 8) | desc[0x34]

* Low byte  = desc[0x34]  (the host's `data_7aede8`, adapter/package selector).
              Validated against minipro's XML: 99.97% (the 1 outlier is a
              same-name collision between two different DLL chips).
* High byte = algo_number, the suffix appended to the per-protocol algorithm
              prefix to name the FPGA bitstream `.alg` file. minipro builds the
              same name in get_algorithm():  algo_table[protocol_id-1] + "%02X".
              e.g. SPI25F + 0x11 -> SPI25F11, NAND_ + 0xC5 -> Nand_C5,
                   EMMC_ + 0x53 -> EMMC_53_18.

algo_number is produced by the host's algorithm-name builder:

    sub_4b45e0 @ 0x4b45e0   (builds "<exe>\algoT76\T7_<prefix><suffix>.alg")
        suffix = (desc[0x35] != 0) ? hex(desc[0x35])     # direct passthrough
                                   : sub_4b3120(...)      # per-protocol tree
    sub_4b3120 @ 0x4b3120   (jump table @ 0x4b3868, switch on protocol_id-1)

The passthrough path (desc[0x35] != 0) covers the NAND / eMMC / parallel-NOR
families (algo lives directly in desc[0x35]: NAND 0xC5, eMMC 0x53, etc.).
The tree path (desc[0x35] == 0) is a faithful port of sub_4b3120 below; it
keys only on protocol_id, desc[0x34], desc[0x38] (code size), desc[0x6c]
(family signature) and desc[0x50].

Faithfulness: across 3159 chips present in BOTH the V12.91 XML and the V13.19
DLL, this function NEVER produces an algorithm number that names a missing
.alg while the XML named a real one (0 genuine bugs). 82.5% reproduce the XML
verbatim; the remaining mismatches are either stale V12.91 entries (the DLL is
newer) or the proto-2 microwire x8/x16 split (V13.19 gives x8 and x16 identical
descriptors, so it emits MW93ALG11 and selects x16 at runtime, while V12.91 XML
kept a separate MW93ALG21 entry). See validate_variant.py for the audit.
"""

def _u32(b, o):
    return int.from_bytes(b[o:o + 4], "little")


def _sub_4b3120(proto, d34, size, fam, d50):
    """Port of sub_4b3120: returns the 2-hex-char algo suffix as an int,
    or None for descriptor field combinations the host leaves undefined."""
    fm = fam & 0xffff00ff
    c = proto - 1
    if c == 0:                                   # proto 1  IIC24C
        al = d34 & 3
        if fm != 0xf6000000:
            if al == 1: return 0x12 if size < 0x8000 else 0x11
            if al == 0: return 0x13
            if al == 2: return 0x14
            return None
        else:
            if al == 1: return 0x62 if size < 0x8000 else 0x61
            if al == 0: return 0x63
            if al == 2: return 0x64
            return None
    if c == 1:                                   # proto 2  MW93ALG (microwire)
        a = d50 & 0xf
        if (d34 & 0x80) == 0:
            if fm == 0xf6000000: return 0x92
            if a == 2: return 0x2A
            return 0x21
        else:
            if fm == 0xf6000000: return 0x91
            if (d34 & 0x20) != 0:
                if a == 1: return 0x69
                if a == 2: return 0x68
                return 0x67
            else:
                if a == 1: return 0x2B
                if a == 2: return 0x1A
                return 0x11
    if c in (2, 0xe):                            # proto 3 / 0xf  SPI25F
        cl = d34 & 3
        if (d34 & 0xf0) == 0x20:
            if cl == 3: return 0x20
            if cl == 2: return 0x21
            return None
        else:
            if cl == 3: return 0x10
            if cl == 2: return 0x11
            if cl == 1: return 0x12
            if cl == 0: return 0x13
            return None
    if c == 4:                                   # proto 5  F29EE
        return 0x76 if fam == 5 else 0x75
    if c == 5:                                   # proto 6  W29F32P
        if (d34 & 0x80) != 0:
            return 0x73 if fam == 5 else 0x71
        else:
            if fam == 5: return 0x72
            if size == 0x80000: return 0x70
            return 0x71
    if c == 6:                                   # proto 7  ROM28P
        if (d34 & 0x10) == 0: return 0x41
        if size == 0x10000: return 0x31
        if size == 0x8000: return 0x32
        return 0x33
    if c == 7:                                   # proto 8  ROM32P
        if fam != 5:
            return {4: 0x12, 3: 0x13, 2: 0x14}.get(d34, 0x11)
        else:
            return {4: 0x22, 3: 0x23, 2: 0x24}.get(d34, 0x21)
    if c == 8:                                   # proto 9  ROM40P
        if fam == 0x28000000: return 0x2A if size == 0x80000 else 0x1A
        if fam == 0xfd000000: return 0x2B if size == 0x80000 else 0x1B
        if fam == 4:          return 0x2C if size == 0x80000 else 0x1C
        return None
    if c == 9:                                   # proto 0xa  R28TO32P
        if (d34 & 0x80) != 0: return 0x42
        if size == 0x10000: return 0x34
        if size == 0x8000: return 0x35
        return 0x36
    if c == 0xa:                                 # proto 0xb  ROM24P
        if (d34 & 0x10) != 0: return 0x43
        if size == 0x800: return 0x3A
        return 0x3B
    if c == 0xc:                                 # proto 0xd  EE28C32P
        return 0x45 if fam == 5 else 0x44
    if c == 0xd:                                 # proto 0xe  RAM32
        return 0x50
    if c == 0xf:                                 # proto 0x10  28F32P
        if d34 in (0x10, 0x11): return 0x7E if fam == 5 else 0x7B
        if d34 == 0x12:         return 0x7F if fam == 5 else 0x7C
        return 0x7D if fam == 5 else 0x7A
    if c == 0x10:                                # proto 0x11  FWH
        a = d34 & 0xf
        if fam == 5: return 0x92 if a == 1 else 0x94
        if fam == 3: return 0x95 if a == 1 else 0x96
        return 0x91 if a == 1 else 0x93
    return 0                                      # default: empty suffix


def algo_number(desc):
    """High byte of `variant` (the .alg suffix). `desc` is the 0x74-byte
    descriptor as bytes. Returns int 0..0xff, or None if undefined."""
    if desc[0x35] != 0:
        return desc[0x35]
    return _sub_4b3120(desc[0x00], desc[0x34], _u32(desc, 0x38),
                       _u32(desc, 0x6c), desc[0x50])


def variant(desc):
    """minipro `variant` field, or None if the algo number is undefined."""
    a = algo_number(desc)
    if a is None:
        return None
    return (a << 8) | desc[0x34]
