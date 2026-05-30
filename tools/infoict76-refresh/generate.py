r"""
Regenerate-and-MERGE infoic.xml's T76 (INFOICT76) database from a V13.19
InfoICT76.dll extraction (v1319_chips.json from extract.py).

Strategy (regression-safe):
  * The existing infoic.xml is preserved byte-for-byte. Every chip already in
    the INFOICT76 database keeps its known-good entry -- nothing is changed.
  * Chips whose base part name is absent from the existing T76 database are
    emitted as NEW <ic> entries with:
      - protocol_id / type / variant / all geometry / chip_id / flags /
        package_details  -> computed from the descriptor (fields.py + variant.py)
      - pin_map / voltages -> CRIBBED from the closest existing sibling
        (same protocol_id + variant + package, then looser), because pin_map is
        not encoded in the descriptor (it indexes the host's per-package pin
        tables -- the remaining RE item).
  * A new chip is SKIPPED (and reported) if its variant is undefined or no
    cribbing sibling exists, so we never emit a broken entry.
  * New entries go in one <manufacturer name="XGPRO_T76_V1319_REFRESH"> block
    inserted at the end of the INFOICT76 database. minipro's -p lookup scans all
    <ic> by name regardless of manufacturer, so placement is safe.

Usage:  python3 generate.py [infoic.xml] [v1319_chips.json] [out.xml]
"""
import re, json, sys, os
from collections import defaultdict, Counter
import fields as F
from variant import variant, algo_number

XML    = sys.argv[1] if len(sys.argv) > 1 else "../../infoic.xml"
JSON   = sys.argv[2] if len(sys.argv) > 2 else "/tmp/v1319_chips.json"
OUT    = sys.argv[3] if len(sys.argv) > 3 else "infoic.refreshed.xml"
ALGDIR = sys.argv[4] if len(sys.argv) > 4 else "../../../xgpro-install/algoT76"

# --- bitstream availability gate: never emit a chip we have no .alg for -------
# (minipro builds the name as algo_table[protocol_id-1] + "%02X"(variant>>8);
#  eMMC appends _18/_33.) A chip whose variant names a missing bitstream cannot
#  be programmed regardless of how correct its other fields are.
_ALGO = ["IIC24C","MW93ALG","SPI25F","AT45D","F29EE","W29F32P","ROM28P","ROM32P",
"ROM40P","R28TO32P","ROM24P","ROM44","EE28C32P","RAM32","SPI25F","28F32P","FWH","T48",
"T40A","T40B","T88V","PIC32X","P18F87J","P16F","P18F2","P16F5X","P16CX","","ATMGA_",
"ATTINY_","AT89P20_","","AT89C_","P87C_","SST89_","W78E_","","","ROM24P","ROM28P",
"RAM32","GAL16","GAL20","GAL22","NAND_","PIC32X","RAM36","KB90","EMMC_","VGA_","CPLD_",
"GEN_","ITE_"]
_VALID_ALG = set()
for _f in os.listdir(ALGDIR):
    if _f.endswith(".alg"):
        _n = _f[:-4]
        for _p in ("T7_", "T76_"):
            if _n.startswith(_p): _n = _n[len(_p):]
        _VALID_ALG.add(_n.upper())

def has_bitstream(d):
    proto = d[0x00]; algo = algo_number(d)
    if algo is None or proto - 1 >= len(_ALGO) or not _ALGO[proto - 1]:
        return False
    pre = _ALGO[proto - 1]
    if proto == 0x31:        # eMMC: EMMC_<algo>_18 or _33
        return (pre + "%02X_18" % algo).upper() in _VALID_ALG or \
               (pre + "%02X_33" % algo).upper() in _VALID_ALG
    return (pre + "%02X" % algo).upper() in _VALID_ALG

def hx(s):
    try: return int(s, 16)
    except Exception: return None

def base_name(n):
    """Part number without package annotation: 'W25Q64BV @DIP8' -> 'W25Q64BV'."""
    return n.split("@")[0].strip()

def is_junk(n):
    """UI placeholders from the 'favorites'/'User' pseudo-manufacturers."""
    low = n.lower()
    return ("favorit" in low or "(user)" in low or "useful" in low
            or n.strip() == "" or low.startswith("my "))

def pkg_token(n):
    """Package token after '@': 'KLM..._8Bit@BGA153' -> 'BGA153'."""
    if "@" not in n: return ""
    return n.split("@", 1)[1].strip().split()[0] if n.split("@", 1)[1].strip() else ""

# --- locate the INFOICT76 database region (everything else is left untouched) -
text = open(XML).read()
m_open = re.search(r'<database\s+type="INFOICT76"', text)
db_start = m_open.start()
db_end = text.index("</database", db_start)        # close of the T76 database

t76 = text[db_start:db_end]

def attr(b, n):
    mm = re.search(r'%s="([^"]*)"' % n, b); return mm.group(1) if mm else None

# pin_map (minipro <maps> index, low byte) is NOT derivable from the T76
# descriptor -- proven: W25Q64BV (proto 3) and ACE24AC02 (proto 1) both have
# desc[0x05]=0 yet use different pin_maps; sub_4d1c60's insertion-test mask
# (keyed on desc[0x05]) is not the maintainer's index source. It is a
# cross-programmer shared layout keyed by chip identity. minipro uses only its
# low byte (BEGIN_TRANS msg[7] + pin-test reporting); it is NOT load-bearing for
# read/write/erase.
#
# Best available sources, in order:
#  1. AUTHORITATIVE: the same chip in the shared INFOIC2PLUS section (verified
#     100% pin_map agreement on chips present in both sections).
#  2. crib from the closest T76 sibling, tightest tier first:
#     (proto,variant,pkg) -> (proto,variant) -> (proto,pkg) -> proto.
#
# The <maps> are shared across all minipro programmer databases; build the
# INFOIC2PLUS index now (defined before the crib indices so it is the top tier).
i2p_pinmap = {}                                  # base name -> pin_map low byte
m_i2p = re.search(r'<database\s+type="INFOIC2PLUS"', text)
if m_i2p:
    i2p = text[m_i2p.start():text.index("</database", m_i2p.start())]
    for b in re.findall(r"<ic\b(.*?)/>", i2p, re.S):
        pm = attr(b, "pin_map"); names = attr(b, "name")
        if pm is None or not names:
            continue
        for nm in names.split(","):
            i2p_pinmap.setdefault(base_name(nm.strip()), hx(pm) & 0xff)

# pinmap_model.json: a descriptor -> pin_map predictor learned from the
# authoritative InfoIC2Plus database (see build_pinmap_model.py). Keyed on the
# feature tuple (proto, desc[0x39]u16, desc[0x6c], desc[0x05], desc[0x04]);
# value = [pin_map_low, agreement, n]. Leave-one-out 96.1% overall, 100% for
# SPI/I2C/NAND/eMMC. Used when its agreement clears PINMAP_MODEL_MIN.
PINMAP_MODEL_MIN = 0.95
try:
    pinmap_model = json.load(open("pinmap_model.json"))
except Exception:
    pinmap_model = {}
def _model_feat(d):
    return "%d:%d:%d:%d:%d" % (d[0x00], F.u(d, 0x39, 2), F.u(d, 0x6c, 4), d[0x05], d[0x04])
def model_pinmap(d):
    e = pinmap_model.get(_model_feat(d))
    if e and e[1] >= PINMAP_MODEL_MIN:
        return e[0], e[1]
    return None, 0.0

existing_bases = set()
crib_pvk = defaultdict(Counter)   # (proto,variant,pkg) -> Counter(pin_map)
crib_pv  = defaultdict(Counter)   # (proto,variant)
crib_pk  = defaultdict(Counter)   # (proto,pkg)
crib_p   = defaultdict(Counter)   # proto
for b in re.findall(r"<ic\b(.*?)/>", t76, re.S):
    names = attr(b, "name")
    if not names: continue
    p = hx(attr(b, "protocol_id") or "0"); v = hx(attr(b, "variant") or "0")
    pm = hx(attr(b, "pin_map") or "0")
    for nm in names.split(","):
        nm = nm.strip(); k = pkg_token(nm)
        existing_bases.add(base_name(nm))
        crib_pvk[(p, v, k)][pm] += 1
        crib_pv[(p, v)][pm] += 1
        crib_pk[(p, k)][pm] += 1
        crib_p[p][pm] += 1

CRIB_TIER = ["i2p", "model", "pvk", "pv", "pk", "p"]
def crib_pinmap(name, d, proto, var, pkg):
    """Pick pin_map, authoritative source first.
    Returns (pin_map, tier, confidence):
      'i2p'   = exact same chip in the shared INFOIC2PLUS section (conf 1.0);
      'model' = the InfoIC2Plus-learned predictor (conf = its agreement);
      pvk/pv/pk/p = crib from the closest T76 sibling (conf = tier agreement).
    pin_map drives only pin-test reporting, never read/write/erase."""
    b = base_name(name)
    if b in i2p_pinmap:
        return i2p_pinmap[b], "i2p", 1.0
    mv, mc = model_pinmap(d)
    if mv is not None:
        return mv, "model", mc
    for tier, c in (("pvk", crib_pvk[(proto, var, pkg)]), ("pv", crib_pv[(proto, var)]),
                    ("pk", crib_pk[(proto, pkg)]), ("p", crib_p[proto])):
        if c:
            val, n = c.most_common(1)[0]
            return val, tier, n / sum(c.values())
    return None, None, 0.0

# --- field order / formatting matching the existing entries -------------------
def emit_ic(name, d, pin_map, volt, pin_note=None):
    a = lambda k, val, w: '          %s="0x%0*X"' % (k, w, val)
    lines = []
    if pin_note:
        lines.append("      <!-- %s -->" % pin_note)
    lines += [
        '      <ic',
        '          name="%s"' % name,
        '          type="%d"' % F.chip_type(d),
        '          protocol_id="0x%02X"' % F.protocol_id(d),
        '          variant="0x%X"' % variant(d),
        a("read_buffer_size",  F.read_buffer_size(d), 2),
        a("write_buffer_size", F.write_buffer_size(d), 2),
        a("code_memory_size",  F.code_memory_size(d), 2),
        a("data_memory_size",  F.data_memory_size(d), 2),
        a("data_memory2_size", F.data_memory2_size(d), 2),
        a("page_size",         F.page_size(d), 4),
        a("pages_per_block",   F.pages_per_block(d), 4),
        '          chip_id="0x%08X"' % F.chip_id(d),
        '          voltages="0x%X"' % volt,
        '          pulse_delay="0x%X"' % F.pulse_delay(d),
        '          flags="0x%08X"' % F.flags(d),
        '          chip_info="0x%04X"' % F.chip_info(d),
        '          pin_map="0x%08X"' % pin_map,
        '          package_details="0x%08X"' % F.package_details(d),
        '          config="NULL"',
        '      />',
    ]
    return "\n".join(lines)

# --- walk the DLL, emit new chips --------------------------------------------
chips = json.load(open(JSON))
new_blocks = []
n_total = n_existing = n_new = 0
skip_variant = skip_crib = 0
tier_counts = Counter()
n_pin_confident = n_pin_review = 0
flagged_by_proto = Counter()
seen_new = set()
for mfr, proto, name, raw in chips:
    d = bytes.fromhex(raw)
    if len(d) < 0x74: continue
    n_total += 1
    if is_junk(name):
        continue
    if base_name(name) in existing_bases:
        n_existing += 1; continue
    if base_name(name) in seen_new:            # de-dup repeated DLL names
        continue
    var = variant(d)
    if var is None or not has_bitstream(d):
        skip_variant += 1; continue
    pin_map, tier, conf = crib_pinmap(name, d, d[0], var, pkg_token(name))
    if pin_map is None:                        # no sibling at all in this protocol
        skip_crib += 1; continue
    # 'i2p' = authoritative same-chip value; 'model' = InfoIC2Plus-learned
    # predictor (>=95% agreement); otherwise the crib confidence. Flag the
    # uncertain ones for review. pin_map is pin-test-reporting only -- never
    # affects read/write.
    confident = tier in ("i2p", "model") or (conf >= 0.95 and tier != "p")
    note = None
    if not confident:
        n_pin_review += 1
        flagged_by_proto[d[0]] += 1
        note = ("pin_map cribbed tier=%s agree=%.1f%% (pin-test reporting only, "
                "not used for read/write) -- verify" % (tier, conf * 100))
    else:
        n_pin_confident += 1
    new_blocks.append(emit_ic(name, d, pin_map, F.voltages(d), note))
    tier_counts[tier] += 1
    seen_new.add(base_name(name)); n_new += 1

block = ('    <manufacturer\n        name="XGPRO_T76_V1319_REFRESH"\n      >\n'
         + "\n".join(new_blocks) + "\n    </manufacturer\n    >\n  ")
merged = text[:db_end] + block + text[db_end:]
open(OUT, "w").write(merged)

print("DLL descriptors          : %d" % n_total)
print("already in T76 database  : %d" % n_existing)
print("NEW chips emitted        : %d" % n_new)
print("  pin_map crib tiers     : " +
      ", ".join("%s=%d" % (t, tier_counts[t]) for t in CRIB_TIER if tier_counts[t]))
print("  pin_map confident      : %d   (flagged for review: %d)" %
      (n_pin_confident, n_pin_review))
print("  pin_map review by proto: " +
      ", ".join("0x%02x=%d" % (p, n) for p, n in flagged_by_proto.most_common()))
print("skipped (no variant/bitstream): %d" % skip_variant)
print("skipped (no proto sibling): %d" % skip_crib)
print("wrote %s (+%d bytes)" % (OUT, len(merged) - len(text)))
