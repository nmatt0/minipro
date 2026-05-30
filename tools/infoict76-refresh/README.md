# infoic.xml refresh tool for T76

> **Status:** the refresh has been **applied** to `infoic.xml` on branch
> `t76-compat-improvements` (2028 new V13.19 chips added; existing entries
> unchanged). Re-run `generate.py` to regenerate. Outstanding item: exact
> `pin_map` for the 875 review-flagged chips (see Open problems).


minipro's bundled `infoic.xml` T76 data is from **XGPro_T76 V12.91** (see the
provenance comment at the top of `infoic.xml`). This tool regenerates / refreshes
it from a newer `InfoICT76.dll` (e.g. V13.19, which pairs with T76 firmware
00.1.17). Extraction, the field map (incl. the `variant`, `flags`,
`package_details` transforms), and a regression-safe MERGE generator are all
done. The one field still not derivable from the descriptor is `pin_map`
(computed by the host from per-package pin tables); the generator cribs it from
an existing same-variant/same-package sibling.

## Files

| File                  | What                                                                 |
| --------------------- | -------------------------------------------------------------------- |
| `extract.py`          | Parse `InfoICT76.dll` -> `/tmp/v1319_chips.json` (35399 descriptors). |
| `variant.py`          | `variant(desc)` -- the `variant`/algorithm transform (RE'd, 0 bugs). |
| `fields.py`           | All other descriptor->infoic.xml field mappings (type, geometry, flags, package_details, ...). |
| `generate.py`         | Regenerate-and-MERGE: keep existing entries, emit new chips -> `infoic.refreshed.xml`. |
| `validate_variant.py` | Audit `variant.py` vs the bundled XML (`I'm WRONG: 0`).              |

## Field map (descriptor offset -> infoic.xml attribute)

Validated on the ~3159-chip V12.91-XML / V13.19-DLL overlap (`fields.py`):

| attribute          | source                         | overlap match |
| ------------------ | ------------------------------ | ------------- |
| type (chip_type)   | `desc[0x08]` u8                | 100%          |
| protocol_id        | `desc[0x00]` u8                | 100%          |
| variant            | `(algo<<8)|desc[0x34]` (variant.py) | 82.5% verbatim, 0 wrong |
| read_buffer_size   | `desc[0x48]` u16               | 100%          |
| write_buffer_size  | `desc[0x4a]` u16               | 99.1%         |
| code_memory_size   | `desc[0x38]` u32               | 100%          |
| data_memory_size   | `desc[0x3c]` u32               | 99.5%         |
| data_memory2_size  | `desc[0x40]` u32               | 100%          |
| page_size          | `desc[0x54]` u32               | 98.8%         |
| pages_per_block    | `desc[0x68]` u32 (high bits intentional) | 99.5% |
| pulse_delay        | `desc[0x58]` u32               | 99.2%         |
| chip_info          | `desc[0x44]` u16               | 99.8%         |
| chip_id            | `desc[0x5b]` u32 BE            | 90.7%         |
| flags              | `desc[0x70]` + per-proto adjust | 99.1%        |
| package_details    | `desc[0x6c]` + family OR       | 95.7%         |
| voltages           | `desc[0x4c]` u16               | 69% (cribbed for new chips) |
| pin_map            | not in descriptor              | cribbed       |

### flags adjustments (from t76_load_chip_to_state @0x4eed10)

* NAND (proto 0x2d): direct (minipro re-ORs `0x800` at send time, t76.c:671).
* eMMC (proto 0x31): clear `0x20` (has_chip_id).
* IIC24C (proto 1): `|= 0x100000` if `desc[0x50]==0`.
* MW93ALG (proto 2): `|= 0x100000` if `(desc[0x34]&0x20)==0`.
* SPI25F (proto 3): `|= 0x48`; `|= 0x100000` unless in the 8b/90/91/9a
  package-table family (a table lookup decides -- not ported, minor).
* proto 4: `|= 0x100048`.

### package_details adjustment

`desc[0x6c]`, then `|= 0x900/0xa00/0xb00` (proto 3/2/1) when
`(value & 0xff00) != 0x500`.

## The `variant` transform (`variant.py`)

    variant = (algo_number << 8) | desc[0x34]

* **Low byte = `desc[0x34]`** (host global `data_7aede8`, adapter/package
  selector). 99.97% of overlap chips.
* **High byte = `algo_number`**, the `.alg` suffix (minipro `get_algorithm()`:
  `algo_table[proto-1] + "%02X"`; e.g. SPI25F+0x11 -> SPI25F11, NAND_+0xC5 ->
  Nand_C5, EMMC_+0x53 -> EMMC_53_18). The host builds the name in
  `t76_build_alg_name @0x4b45e0`: if `desc[0x35] != 0` then `algo = desc[0x35]`
  (NAND/eMMC/parallel-NOR passthrough); else `algo = t76_algo_suffix_tree
  @0x4b3120`, a per-protocol tree (jump table `@0x4b3868`) keyed on
  `protocol_id`, `desc[0x34]`, `desc[0x38]`, `desc[0x6c]`, `desc[0x50]`.

**`validate_variant.py`:** over 3159 overlap chips the port produces **0 wrong
algorithms** (never names a missing `.alg` where the XML named a real one); 82.5%
reproduce the XML verbatim; the rest are stale V12.91 entries (134) or the
proto-2 microwire x8/x16 split (419: V13.19 gives x8/x16 identical descriptors,
emits MW93ALG11 and runtime-selects x16). HW-validated chips exact:
ZB25VQ64A/MX25L12845E/W25Q64BV 0x1102, IS24C16 0x1300, W29N02GZ 0xc500,
KLM8G1GEAC 0x5300.

## The generator (`generate.py`) — regenerate-and-MERGE

Regression-safe: the existing `infoic.xml` is preserved byte-for-byte; every
chip already in the INFOICT76 database keeps its known-good entry. Chips whose
base part name is absent are emitted as new `<ic>` entries:

* protocol / type / variant / geometry / chip_id / flags / package_details /
  voltages — **computed from the descriptor** (fields.py + variant.py);
* **pin_map** -- minipro's low byte is the `<maps>` index, used **only by the
  pin-test command's host-side reporting** (the firmware does the actual
  detection via opcode `0x3E`); it is never used for read/write/erase.
  **It is provably NOT derivable from the T76 descriptor:** W25Q64BV (proto 3)
  and ACE24AC02 (proto 1) both have `desc[0x05]=0` (the vendor's pin-layout
  selector feeding `sub_4d1c60`) yet use different pin_maps -- so that
  insertion-test mask is not the maintainer's index source. pin_map is a
  cross-programmer shared layout keyed by chip *identity*, living in XGecu's
  other database (InfoIC2.dll for T48/T56), which is not shipped here. Sources,
  best first:
  1. **i2p (authoritative)**: same chip in the shared INFOIC2PLUS section of
     infoic.xml (verified 100% pin_map agreement on the 16792 chips present in
     both sections). Covers new chips that already existed for the older
     programmers.
  2. **crib** from the closest T76 sibling: `(proto,variant,pkg)` ->
     `(proto,variant)` -> `(proto,pkg)` -> `proto`, with a **confidence** =
     fraction of that tier's siblings that agree.

  Each entry is emitted confident or flagged:
  - **confident** (i2p, or agree >=95% and tier not proto-only): leave-one-out
    accuracy **99.2-99.8%** (effectively exact).
  - **flagged** (`<!-- pin_map cribbed tier=.. agree=..% ... verify -->`): ~55%
    accurate; **99% of all crib errors land here**, so review these if you use
    pin-test. Read/write is unaffected either way. Truly-exact values for the
    flagged set require the V13.x InfoIC2.dll (T48/T56), not available here.

A new chip is emitted only if its variant is defined **and resolves to a `.alg`
bitstream present in `algoT76/`** (a chip with no available bitstream cannot be
programmed, so it is skipped, not emitted broken). New entries go in one
`<manufacturer name="XGPRO_T76_V1319_REFRESH">` at the end of the INFOICT76
database (minipro's `-p` scans all `<ic>` by name regardless of manufacturer).

Last run: 30172 chips preserved, **2028 new chips emitted** (all resolve to a
real `.alg`), 92 skipped for lack of an available bitstream (e.g. the MW93ALG92
SOT23-6 microwire family). pin_map: **1153 confident (99.8% accurate), 875
flagged for review** (almost all proto 3/2/0x12 -- new variants and parallel
wide packages). minipro parses the result (`-l` lists 34896 devices, +2088); the
5 hardware-tested chips still resolve, as do newly-added parts (GD25LX64J,
S-24C02D, S-93A46B, ...).

## Open problems (remaining work)

1. **Exact `pin_map` for the 875 flagged chips** (pin-test reporting only; read/
   write already correct). RE'd to a dead end *from this data*: pin_map is not a
   function of the InfoICT76.dll descriptor (counterexample above). The
   authoritative source is the **V13.x InfoIC2.dll** (the T48/T56 chip database)
   -- the same new chips appear there with their shared pin_map. Obtaining and
   extracting that DLL (same walker as `extract.py`) and matching by chip name
   would resolve the flagged set exactly. The confidence flag already marks
   precisely which entries need it.
2. **`voltages`** is `desc[0x4c]` at ~69% overall (82-97% for SPI/I2C/MW93) --
   could be refined further.
3. **Microwire x16 caveat:** new 93Cxx parts get the x8 algo (0x11); x16 is a
   runtime selection in the host, not in `variant`.
4. **On-hardware confirmation:** read a sample newly-added chip on the T76.

## Usage

```
cp /path/to/InfoICT76.dll .
python3 extract.py            # -> /tmp/v1319_chips.json
python3 validate_variant.py   # audit variant.py vs ../../infoic.xml (I'm WRONG: 0)
python3 generate.py           # -> infoic.refreshed.xml (merge; existing entries untouched)
```

`variant.py` / `fields.py` expose per-field functions taking the 0x74-byte
descriptor (bytes); `generate.py` composes them.
