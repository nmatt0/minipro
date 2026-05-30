# infoic.xml refresh tool for T76 (work in progress)

minipro's bundled `infoic.xml` T76 data is from **XGPro_T76 V12.91** (see the
provenance comment at the top of `infoic.xml`). This tool is for regenerating /
refreshing it from a newer `InfoICT76.dll` (e.g. V13.19, which pairs with T76
firmware 00.1.17). It is **not finished**: the extraction and most field mappings
are solved; the `variant`/`flags`/`pin_map`/`package_details` transform is not.

## Status

**Done — `extract.py`** parses `InfoICT76.dll` (place it alongside the script) and
walks the chip database directly from the PE: manufacturer table at VA `0x10172790`
(stride `0x4c`, count `0xAD`=173), chip-array pointer at `mfr+0x44`, count at
`mfr+0x48`, each chip a `0x74`-byte descriptor (name at `+0x0C`, protocol at `+0x00`).
V13.19 yields 35399 descriptors / 28408 unique names. Output: `v1319_chips.json`
of `[mfr, protocol_id, name, raw_hex]`.

**Done — field map (descriptor offset -> infoic.xml attribute).** Validated across
~24k overlap chips (chips present in both the V12.91 XML and the V13.19 DLL):

| infoic.xml attr     | source in 0x74 descriptor        | overlap match |
| ------------------- | -------------------------------- | ------------- |
| protocol_id         | `desc[0x00]` u8                  | ~82% (see note) |
| code_memory_size    | `desc[0x38]` u32 LE              | 99.9%         |
| read_buffer_size    | `desc[0x48]` u16 LE              | ~79%          |
| write_buffer_size   | `desc[0x4a]` u16 LE              | 99.5%         |
| pulse_delay         | `desc[0x58]` u16 LE              | ~87%          |
| chip_info           | `desc[0x44]` u16 LE              | 97%           |
| chip_id             | `desc[0x5c..0x5e]` 3 bytes MSB-first | ~58% (see note) |
| page_size           | TBD (collides with write_buffer in samples) | — |
| data_memory_size/2  | TBD                              | —             |

## Open problems (the remaining work)

1. **`variant` / `flags` / `pin_map` / `package_details` are computed, not copied.**
   They do not appear verbatim in the descriptor. `variant` is critical: it selects
   the FPGA algorithm/bitstream, so a wrong value reproduces the all-zeros/garbage
   READID failures that the T76 read paths were written to fix. No single descriptor
   field predicts `variant` (best ~79% group-purity at `desc[0x2e]`/`[0x32..0x35]`),
   so it is a multi-field function. Two ways forward:
   - Decode how the host derives the algorithm from the descriptor
     (`t76_load_chip_to_state @0x4eed10` + the algorithm-selection path) in Binary Ninja.
   - Data-driven: with precise (per-package) name matching, derive the function from
     the ~24k (descriptor, XML-variant) overlap pairs and validate it reproduces the
     V12.91 XML exactly before applying to new chips.

2. **Name matching must be exact per package.** `analyze_transform.py` currently
   normalizes names and takes first-wins, which collides package variants and is the
   main reason chip_id/read_buffer match rates look low and protocol_id shows
   impossible values (e.g. 0x36) — those are mispaired chips, not a real transform
   bug. Fix the matching first; the direct-field rates should approach 100%.

3. **Possible protocol-id renumbering** between V12.91 and V13.19 — confirm before
   trusting `desc[0x00]` directly (or it may just be the matching noise from #2).

## Intended final flow (regenerate-and-MERGE, no regression)

1. `extract.py` -> all V13.19 descriptors.
2. For chips already in `infoic.xml`: keep their existing `variant`/`flags`/etc.
   (known-good), optionally refresh the validated direct fields.
3. For NEW chips: fill direct fields from the descriptor; derive `variant` (formula
   from #1, or crib from a sibling in the same algorithm family).
4. Emit grouped `<device>` entries (same package-variant grouping as the current XML).
5. Validate: minipro parses it; the known hardware-tested chips (ZB25VQ64A,
   MX25L12845E, S29GL512N, W29N02GZ, KLM8G1GEAC) still resolve with correct params
   and read on hardware.

## Usage (extraction only, today)

```
cp /path/to/InfoICT76.dll .
python3 extract.py            # -> v1319_chips.json (+ stats on stderr)
python3 analyze_transform.py  # field-map validation + variant-predictor search
                              # (expects ../../infoic.xml and v1319_chips.json)
```
