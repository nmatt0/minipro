import struct, sys
DLL="InfoICT76.dll"
d=open(DLL,"rb").read()
# --- minimal PE parse: ImageBase + sections for VA->file-offset ---
e_lfanew=struct.unpack_from("<I",d,0x3C)[0]
assert d[e_lfanew:e_lfanew+4]==b"PE\0\0"
coff=e_lfanew+4
num_sec=struct.unpack_from("<H",d,coff+2)[0]
opt_sz=struct.unpack_from("<H",d,coff+16)[0]
opt=coff+20
magic=struct.unpack_from("<H",d,opt)[0]  # 0x10b PE32
image_base=struct.unpack_from("<I",d,opt+28)[0]
sec_tbl=opt+opt_sz
secs=[]
for i in range(num_sec):
    o=sec_tbl+i*40
    name=d[o:o+8].rstrip(b"\0").decode("latin1")
    vsize,vaddr,rawsize,rawptr=struct.unpack_from("<IIII",d,o+8)
    secs.append((vaddr,vsize,rawptr,rawsize,name))
print(f"ImageBase=0x{image_base:x} magic=0x{magic:x} sections={[s[4] for s in secs]}",file=sys.stderr)
def va2off(va):
    rva=va-image_base
    for vaddr,vsize,rawptr,rawsize,name in secs:
        if vaddr<=rva<vaddr+max(vsize,rawsize):
            return rawptr+(rva-vaddr)
    return None
def ru32(va):
    o=va2off(va); return struct.unpack_from("<I",d,o)[0] if o is not None else None
def rbytes(va,n):
    o=va2off(va); return d[o:o+n] if o is not None else b""
# --- walk manufacturer table ---
MFR_BASE=0x10172790; MFR_STRIDE=0x4c; MFR_COUNT=0xAD; CHIP_SZ=0x74
chips=[]
for i in range(MFR_COUNT):
    m=MFR_BASE+i*MFR_STRIDE
    mfr_name=rbytes(m+4,12).split(b"\0")[0].decode("latin1","replace")
    arr=ru32(m+0x44); cnt=ru32(m+0x48)
    if not arr or cnt is None or cnt>20000: continue
    for j in range(cnt):
        c=arr+j*CHIP_SZ
        raw=rbytes(c,CHIP_SZ)
        if len(raw)<CHIP_SZ: continue
        proto=raw[0]
        name=raw[0x0C:0x34].split(b"\0")[0].decode("latin1","replace")
        chips.append((mfr_name,proto,name,raw.hex()))
print(f"total chips extracted: {len(chips)}",file=sys.stderr)
import json
json.dump(chips,open("/tmp/v1319_chips.json","w"))
# print protocol histogram + sample
from collections import Counter
h=Counter(c[1] for c in chips)
print("proto histogram:",{f"0x{k:02x}":v for k,v in sorted(h.items())},file=sys.stderr)
