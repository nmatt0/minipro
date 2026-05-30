r"""
Map a V13.19 InfoICT76.dll 0x74-byte chip descriptor to minipro infoic.xml
fields. RE'd from t76_load_chip_to_state @0x4eed10 (the host's chip-load
routine) + validated against the V12.91 XML overlap set.

DIRECT copies (offset -> attribute), validated >=99% on the overlap set:

    type (chip_type)   desc[0x08]  u8
    protocol_id        desc[0x00]  u8
    read_buffer_size   desc[0x48]  u16 LE
    write_buffer_size  desc[0x4a]  u16 LE
    code_memory_size   desc[0x38]  u32 LE
    data_memory_size   desc[0x3c]  u32 LE
    data_memory2_size  desc[0x40]  u32 LE
    page_size          desc[0x54]  u32 LE
    pages_per_block    desc[0x68]  u32 LE   (semantics overloaded per protocol)
    pulse_delay        desc[0x58]  u32 LE   (NAND overloads it as page+spare geometry)
    chip_info          desc[0x44]  u16 LE
    chip_id            desc[0x5b]  u32 BE   (= leading 0 byte + 3 ID bytes 0x5c..0x5e)
    variant            see variant.py

COMPUTED (see functions below):

    flags              desc[0x70] + per-protocol post-load adjustments
    package_details    desc[0x6c] + per-protocol family-signature OR
    voltages           desc[0x4c] (low ~69% direct; refine / crib for new chips)
    pin_map            NOT in the descriptor -- computed by the host from the
                       per-package pin tables (10 tables @ 56-byte stride, BGA
                       label table @0x6bd870, adapter table @0x684d90). Not yet
                       ported; the generator cribs it from an existing XML
                       sibling instead. THIS IS THE REMAINING RE WORK.
"""

def u(b, o, n):
    return int.from_bytes(b[o:o + n], "little")

def ube(b, o, n):
    return int.from_bytes(b[o:o + n], "big")


# ----- direct fields -------------------------------------------------------
def chip_type(d):         return d[0x08]   # infoic.xml `type` (minipro device->chip_type), 100%
def protocol_id(d):       return d[0x00]
def read_buffer_size(d):  return u(d, 0x48, 2)
def write_buffer_size(d): return u(d, 0x4a, 2)
def code_memory_size(d):  return u(d, 0x38, 4)
def data_memory_size(d):  return u(d, 0x3c, 4)
def data_memory2_size(d): return u(d, 0x40, 4)
def page_size(d):         return u(d, 0x54, 4)
def pages_per_block(d):   return u(d, 0x68, 4)
def pulse_delay(d):       return u(d, 0x58, 4)   # u32: NAND overloads it as page+spare geometry
def chip_info(d):         return u(d, 0x44, 2)
def chip_id(d):           return ube(d, 0x5b, 4)
def voltages(d):          return u(d, 0x4c, 2)


# ----- flags (data_7aee18) -------------------------------------------------
# desc[0x70], with the post-load adjustments from t76_load_chip_to_state.
# Semantic bits (minipro database.c): 0x20=has_chip_id, 0x4000=off_protect_before,
# 0x8000=protect_after, 0x40000=lock_bit_wo, 0x80000=calibration,
# 0x300000=prog_support (>>20). raw_flags is sent verbatim in BEGIN_TRANS
# (t76.c:561), so it matters. NOTE minipro itself re-ORs 0x800 for NAND at send
# time (t76.c:671) -- including it here is idempotent/harmless.
def flags(d, model=8):                 # model 8 = T76, 6 = T56
    p = d[0x00]
    v = u(d, 0x70, 4)
    if p == 0x2d:                      # NAND: direct (minipro re-ORs 0x800 at send time, t76.c:671)
        return v
    if p == 0x31:                      # eMMC: clear has_chip_id (0x20)
        return v & ~0x20
    if p == 1:                         # IIC24C: |= 0x100000 (prog_support) if desc[0x50]==0
        return v | (0x100000 if d[0x50] == 0 else 0)
    if p == 2:                         # MW93ALG: |= 0x100000 if (desc[0x34] & 0x20)==0
        return v | (0x100000 if (d[0x34] & 0x20) == 0 else 0)
    if p == 3:                         # SPI25F: |= 0x48; |= 0x100000 unless in the
        v |= 0x48                      # 8b/90/91/9a package-table family (then a table
        if d[0x35] not in (0x8b, 0x90, 0x91, 0x9a):   # lookup decides -- not ported).
            v |= 0x100000
        return v
    if p == 4:                         # 0xf/4 common tail: |= 0x100048 (model != T56)
        return v | (0x100048 if model != 6 else 0)
    return v                           # incl. proto 0xf (28F32P/SPI25F2): direct


# ----- package_details (data_7aee14) --------------------------------------
# desc[0x6c], with the family-signature OR from the chip-load tail
#   (ebx_1 & 0xff00) != 0x500  ->  |= 0x900 (per-proto 0xa00/0xb00 for proto 2/1).
# High SMD bit (0x80000000) and NAND 0xc0000000 edge cases are left as-is
# (the generator's MERGE keeps existing entries; these affect few new chips).
_FAM_OR = {1: 0xb00, 2: 0xa00, 3: 0x900}
def package_details(d):
    p = d[0x00]
    v = u(d, 0x6c, 4)
    if p in _FAM_OR and (v & 0xff00) != 0x500:
        v |= _FAM_OR[p]
    return v
