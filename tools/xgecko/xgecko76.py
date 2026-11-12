#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 rrr/cils. Upstream: https://github.com/rrr-cils/XGecko76
"""Unpack and repack XGecu T76 firmware update files (UpdateT76.Dat).

    xgecko76.py -u UpdateT76.Dat -o fw.bin
    xgecko76.py -p my_fw.bin -k UpdateT76.Dat -o my_update.Dat

File format (all integers little-endian)
----------------------------------------
header, 16 bytes:
    u32  0xF076_xxxx    xxxx = firmware version (0x0113 = 1.19)
    u32  file CRC       crc32(all blocks, init 0xFFFFFFFF), see crc32() below
    u32  0
    u32  n              number of blocks

n blocks of 0x114 bytes; block i carries firmware bytes i*256 .. i*256+255:
    u32  W              crc32(P, init 0)
    u16  F              per-block cipher seed (any value works)
    10   zero bytes
    260  C              encrypted P

P = u32(i * 0x100) || 256 firmware bytes    (an address marker, then the data)

Cipher, for p = 0..259, with key table K (515 bytes):
    G    = F & 0xFF if p % 4 == 0, F >> 8 if p % 4 == 1, else 0
    C[p] = ror3(P[p] ^ G) ^ K[((F & 0xFF) + (F >> 8)) % 256 + p]

The key is not stored here. learn_key() recovers it from any vendor .Dat, and
every block's address marker and CRC prove the result.
"""
import argparse, hashlib, os, struct, sys

BLK, DLEN, NSLOT = 0x114, 260, 515
MAX_BLOCKS = 0x49F00 // 256   # the updater writes a fixed marker page at 0x49F00

VERBOSE = False
def log(msg, tag="*"):
    """Verbose output: [*] progress, [+] check passed, [!] notable (a repair)."""
    if VERBOSE:
        print(f"[{tag}] {msg}", file=sys.stderr)

def ver(v):     return f"{v >> 8}.{v & 0xFF:02d}"
def nblocks(n): return f"{n} block{'s' * (n != 1)}"

ror3 = lambda x: ((x >> 3) | (x << 5)) & 0xFF
rol3 = lambda x: ((x << 3) | (x >> 5)) & 0xFF

CRC_TABLE = []
for n in range(256):
    for _ in range(8):
        n = (n >> 1) ^ 0xEDB88320 if n & 1 else n >> 1
    CRC_TABLE.append(n)

def crc32(data, init):
    """Reflected CRC-32 (zlib polynomial) with no final XOR."""
    c = init
    for b in data:
        c = (c >> 8) ^ CRC_TABLE[(c ^ b) & 0xFF]
    return c

def g(f, p):      return (f & 0xFF) if p % 4 == 0 else (f >> 8) if p % 4 == 1 else 0
def start(f):     return ((f & 0xFF) + (f >> 8)) & 0xFF

def encrypt(K, P, f):
    s = start(f)
    return bytes(ror3(P[p] ^ g(f, p)) ^ K[s + p] for p in range(DLEN))

def decrypt(K, C, f):
    s = start(f)
    return bytes(rol3(C[p] ^ K[s + p]) ^ g(f, p) for p in range(DLEN))

def read(path):
    try:
        return open(path, "rb").read()
    except OSError as e:
        sys.exit(f"{path}: {e.strerror}")

def write(path, data):
    try:
        open(path, "wb").write(data)
    except OSError as e:
        sys.exit(f"{path}: {e.strerror}")

def parse(dat, name):
    """-> (version, [(W, F, C), ...]); checks the header and file CRC."""
    if len(dat) < 16:
        sys.exit(f"{name}: not a T76 update file")
    magic, fcrc, _, n = struct.unpack_from("<IIII", dat)
    if magic >> 16 != 0xF076 or len(dat) != 16 + n * BLK:
        sys.exit(f"{name}: not a T76 update file")
    if crc32(dat[16:], 0xFFFFFFFF) != fcrc:
        sys.exit(f"{name}: file CRC mismatch, the file is damaged")
    blocks = []
    for i in range(n):
        b = dat[16 + i * BLK:16 + (i + 1) * BLK]
        W, F = struct.unpack_from("<IH", b)
        blocks.append((W, F, b[16:]))
    log(f"{os.path.basename(name)}: firmware {ver(magic & 0xFFFF)}, {nblocks(n)}")
    log(f"file CRC {fcrc:08x} ok", "+")
    return magic & 0xFFFF, blocks

def walsh(a):
    """Walsh-Hadamard transform of 256 values (turns XOR correlation into products)."""
    a, h = list(a), 1
    while h < 256:
        for i in range(0, 256, 2 * h):
            for j in range(i, i + h):
                a[j], a[j + h] = a[j] + a[j + h], a[j] - a[j + h]
        h *= 2
    return a

def learn_key(blocks):
    """Recover K from the ciphertext of one vendor file.

    1. Statistics: undo the seed XOR, and each key slot is left XORing the same
       (rotated) firmware byte stream. So every slot's byte histogram is one
       shared histogram, XOR-shifted by that slot's key byte. Align each slot
       to a common reference (best XOR correlation) and refine.
    2. One global XOR constant is still free. Every P[0] is 0 (the address low
       byte), so the majority vote over all blocks fixes it.
    3. A few slots can still be wrong. A wrong key byte corrupts exactly one
       plaintext byte, so the CRC syndrome crc(P) ^ W of a failing block gives
       the position p and the error e. The slot is start + p and its correct
       value is K ^ ror3(e). Apply the most-voted fix until every block passes.
    """
    hist = [[0] * 256 for _ in range(NSLOT)]
    for _, f, C in blocks:
        for p in range(DLEN):
            hist[start(f) + p][C[p] ^ ror3(g(f, p))] += 1
    used = [t for t in range(NSLOT) if any(hist[t])]
    if len(used) < NSLOT:
        sys.exit("too few blocks to learn the key from; add -k UpdateT76.Dat (a vendor file)")
    log(f"learning key from {len(blocks) * DLEN} encrypted bytes (all {NSLOT} slots used)")
    K, ref = [0] * NSLOT, max(hist, key=sum)
    for _ in range(5):
        R, total = walsh(ref), [0] * 256
        for t in used:
            corr = walsh([a * b for a, b in zip(walsh(hist[t]), R)])  # corr[k] = sum hist[v^k]*ref[v]
            K[t] = max(range(256), key=corr.__getitem__)
            for v in range(256):
                total[v] += hist[t][v ^ K[t]]
        ref = total

    votes = {}
    for _, f, C in blocks:
        x = C[0] ^ K[start(f)] ^ ror3(g(f, 0))
        votes[x] = votes.get(x, 0) + 1
    x = max(votes, key=votes.get)
    K = bytearray(k ^ x for k in K)
    log(f"global constant {x:#04x} ({votes[x]}/{len(blocks)} blocks agree)")

    # syndrome[p][S] = e: plaintext error e at position p changes the CRC by S
    syndrome = []
    for p in range(DLEN):
        bits = [crc32(bytes(p) + bytes([1 << j]) + bytes(DLEN - p - 1), 0) for j in range(8)]
        syndrome.append({_xor(bits, e): e for e in range(1, 256)})
    fixed = 0
    for _ in range(64):
        votes = {}
        for W, f, C in blocks:
            S = crc32(decrypt(K, C, f), 0) ^ W
            for p in range(DLEN) if S else ():
                if S in syndrome[p]:
                    fix = (start(f) + p, K[start(f) + p] ^ ror3(syndrome[p][S]))
                    votes[fix] = votes.get(fix, 0) + 1
                    break
        if not votes:
            break
        t, v = max(votes, key=votes.get)
        log(f"CRC repaired key slot {t}: {K[t]:#04x} -> {v:#04x} ({votes[(t, v)]} blocks)", "!")
        K[t] = v
        fixed += 1
    log(f"key recovered ({fixed} slot{'s' * (fixed != 1)} repaired)", "+")
    return bytes(K)

def _xor(bits, e):
    s = 0
    for j in range(8):
        if e >> j & 1:
            s ^= bits[j]
    return s

def unpack_blocks(K, blocks):
    """Decrypt every block; exit unless every address marker and CRC is right."""
    image = bytearray()
    for i, (W, f, C) in enumerate(blocks):
        P = decrypt(K, C, f)
        if P[:4] != struct.pack("<I", i * 0x100) or crc32(P, 0) != W:
            sys.exit(f"block {i} does not decrypt correctly")
        image += P[4:]
    log(f"{nblocks(len(blocks))} checked: address markers and CRC-32 correct", "+")
    return bytes(image)

def unpack(dat_path, out_path, key_path=None):
    """Decrypt dat_path. The key is learned from dat_path itself, or from key_path
    (a vendor file) when dat_path is too small to learn it from."""
    version, blocks = parse(read(dat_path), dat_path)
    kblocks = parse(read(key_path), key_path)[1] if key_path else blocks
    image = unpack_blocks(learn_key(kblocks), blocks)
    write(out_path, image)
    log(f"wrote {os.path.basename(out_path)}, sha256 {hashlib.sha256(image).hexdigest()}")
    print(f"{out_path}: firmware {ver(version)}, {len(image)} bytes, "
          f"{nblocks(len(blocks))} verified")

def pack(fw_path, dat_path, out_path):
    """Encrypt fw_path using the key, seeds and version of the vendor file dat_path.

    Reusing the vendor seeds means repacking an unmodified image gives back the
    vendor file byte for byte."""
    version, vblocks = parse(read(dat_path), dat_path)
    K = learn_key(vblocks)
    unpack_blocks(K, vblocks)                      # proves the key on the vendor file
    image = read(fw_path)
    n = -(-len(image) // 256)
    if not 0 < n <= MAX_BLOCKS:
        sys.exit(f"firmware must be 1 to {MAX_BLOCKS * 256} bytes")
    log(f"packing {os.path.basename(fw_path)}: {len(image)} bytes -> {nblocks(n)}"
        f"{f' ({n * 256 - len(image)} zero bytes of padding)' if len(image) % 256 else ''}; "
        f"seeds and version {ver(version)} from {os.path.basename(dat_path)}")
    image += bytes(n * 256 - len(image))           # zero-pad the last block, as the vendor does
    body = bytearray()
    for i in range(n):
        P = struct.pack("<I", i * 0x100) + image[i * 256:(i + 1) * 256]
        f = vblocks[i % len(vblocks)][1]
        body += struct.pack("<IH", crc32(P, 0), f) + bytes(10) + encrypt(K, P, f)
    dat = struct.pack("<IIII", 0xF0760000 | version, crc32(body, 0xFFFFFFFF), 0, n) + body
    log("reading the new file back")
    _, blocks = parse(dat, out_path)
    if unpack_blocks(K, blocks) != image:          # read back what we wrote
        sys.exit("internal error: the packed file does not read back; nothing written")
    write(out_path, dat)
    log(f"wrote {os.path.basename(out_path)}, sha256 {hashlib.sha256(dat).hexdigest()}")
    print(f"{out_path}: firmware {ver(version)}, {nblocks(n)}")

def main():
    ap = argparse.ArgumentParser(
        usage="%(prog)s -u UpdateT76.Dat -o fw.bin\n"
              "       %(prog)s -p my_fw.bin -k UpdateT76.Dat -o my_update.Dat",
        description="Unpack and repack XGecu T76 firmware update files (UpdateT76.Dat).",
        epilog="examples:\n"
               "  %(prog)s -u UpdateT76.Dat -o fw.bin\n"
               "  %(prog)s -p my_fw.bin -k UpdateT76.Dat -o my_update.Dat",
        formatter_class=lambda prog: argparse.RawDescriptionHelpFormatter(prog, max_help_position=30))
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("-u", "--unpack", metavar="UpdateT76.Dat", help="decrypt an update file to a firmware image")
    mode.add_argument("-p", "--pack", metavar="my_fw.bin", help="encrypt a firmware image into an update file")
    ap.add_argument("-k", "--key", metavar="UpdateT76.Dat",
                    help="vendor file to take the key from; needed for -p")
    ap.add_argument("-o", "--output", metavar="FILE", required=True,
                    help="file to write: fw.bin (-u) or my_update.Dat (-p)")
    ap.add_argument("-v", "--verbose", action="store_true", help="show each step")
    if len(sys.argv) == 1:
        return ap.print_help()
    a = ap.parse_args()
    global VERBOSE
    VERBOSE = a.verbose
    if a.pack and not a.key:
        ap.error("-p needs -k UpdateT76.Dat (a vendor file supplies the key)")
    if os.path.exists(a.output) and any(f and os.path.exists(f) and os.path.samefile(f, a.output)
                                        for f in (a.unpack, a.pack, a.key)):
        ap.error(f"-o {a.output} is also an input; refusing to overwrite it")
    if a.unpack:
        unpack(a.unpack, a.output, a.key)
    else:
        pack(a.pack, a.key, a.output)

if __name__ == "__main__":
    main()
