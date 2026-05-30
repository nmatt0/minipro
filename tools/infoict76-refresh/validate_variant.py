"""
Audit variant.py against minipro's bundled infoic.xml.

For every chip present in BOTH the V12.91 XML and the extracted V13.19 DLL
(matched on name + protocol_id + code_memory_size to defeat same-name
collisions), compute the variant and compare. Mismatches are classified by
whether the computed vs XML algorithm number names a real .alg bitstream:

  - "I'm wrong"   : XML names a real .alg, computed one does not  -> a bug.
  - "XML stale"   : computed names a real .alg, XML's does not    -> DLL is newer.
  - "version diff": both name a real .alg                         -> e.g. 93Cxx x16.

Run from this directory; expects ../../infoic.xml, ../../../xgpro-install/algoT76
and v1319_chips.json (produced by extract.py). Paths can be overridden via argv.
"""
import re, json, os, sys
from collections import Counter, defaultdict
from variant import variant, algo_number, _u32

XML   = sys.argv[1] if len(sys.argv) > 1 else "../../infoic.xml"
JSON  = sys.argv[2] if len(sys.argv) > 2 else "/tmp/v1319_chips.json"  # where extract.py writes
ALGDIR= sys.argv[3] if len(sys.argv) > 3 else "../../../xgpro-install/algoT76"

# minipro algo_table (protocol_id -> prefix), index = protocol_id-1
ALGO = ["IIC24C","MW93ALG","SPI25F","AT45D","F29EE","W29F32P","ROM28P","ROM32P",
        "ROM40P","R28TO32P","ROM24P","ROM44","EE28C32P","RAM32","SPI25F","28F32P",
        "FWH","T48","T40A","T40B","T88V","PIC32X","P18F87J","P16F","P18F2","P16F5X",
        "P16CX","","ATMGA_","ATTINY_","AT89P20_","","AT89C_","P87C_","SST89_","W78E_",
        "","","ROM24P","ROM28P","RAM32","GAL16","GAL20","GAL22","NAND_","PIC32X",
        "RAM36","KB90","EMMC_","VGA_","CPLD_","GEN_","ITE_"]

valid = set()
for f in os.listdir(ALGDIR):
    if f.endswith(".alg"):
        n = f[:-4]
        for p in ("T7_", "T76_"):
            if n.startswith(p): n = n[len(p):]
        valid.add(n.upper())

def algoname(proto, algo):
    if algo is None or proto - 1 >= len(ALGO) or not ALGO[proto - 1]:
        return None
    return (ALGO[proto - 1] + "%02X" % algo).upper()

def hx(s):
    try: return int(s, 16)
    except Exception: return None

txt = open(XML).read()
def attr(b, n):
    m = re.search(r'%s="([^"]*)"' % n, b); return m.group(1) if m else None
xml = defaultdict(set)
for b in re.findall(r"<ic\b(.*?)/>", txt, re.S):
    names = attr(b, "name"); var = attr(b, "variant")
    if not names or var is None: continue
    key_p = hx(attr(b, "protocol_id")); key_c = hx(attr(b, "code_memory_size"))
    for nm in names.split(","):
        xml[(nm.strip(), key_p, key_c)].add(hx(var))

chips = json.load(open(JSON))
tot = ok = wrong = stale = vdiff = 0
seen = set(); bugs = []
for mfr, proto, name, raw in chips:
    d = bytes.fromhex(raw)
    if len(d) < 0x74: continue
    key = (name, d[0], _u32(d, 0x38))
    if key not in xml or key in seen: continue
    seen.add(key); tot += 1
    g = variant(d); vs = xml[key]
    if g in vs:
        ok += 1; continue
    gv = algoname(d[0], algo_number(d)) in valid
    xv = any(algoname(d[0], (x >> 8) & 0xff) in valid for x in vs)
    if xv and not gv:
        wrong += 1; bugs.append((name, d[0], sorted(hex(x) for x in vs), hex(g) if g else None))
    elif gv and not xv:
        stale += 1
    else:
        vdiff += 1

print("chips compared        : %d" % tot)
print("exact XML match       : %d (%.2f%%)" % (ok, 100 * ok / tot))
print("mismatch, I'm WRONG   : %d   <-- must be 0" % wrong)
print("mismatch, XML stale   : %d" % stale)
print("mismatch, version diff: %d" % vdiff)
for x in bugs[:20]:
    print("  BUG:", x)
