import json,re,struct
from collections import defaultdict,Counter
chips=json.load(open("/tmp/v1319_chips.json"))
def norm(s): return re.sub(r'\s+','',s).lower()
# index DLL chips by normalized name -> raw bytes (first wins)
dll={}
for mfr,proto,name,raw in chips:
    k=norm(name)
    if k and k not in dll: dll[k]=(proto,bytes.fromhex(raw))
# parse XML: each <device ...> block -> attrs + names
xml=open("../../infoic.xml").read()
blocks=re.findall(r'<(?:device|custom)\b[^>]*?/?>', xml, re.S) or re.findall(r'name="[^"]*"[^/]*?/>', xml, re.S)
# more robust: split on name= ... config=
entries=[]
for m in re.finditer(r'(name="[^"]*"[\s\S]*?config="[^"]*"\s*/>)', xml):
    blk=m.group(1)
    g=lambda a: (re.search(a+r'="([^"]*)"',blk) or [None,None])[1]
    names=g("name").split(",")
    def gi(a):
        v=g(a); return int(v,16) if v else None
    attrs=dict(variant=gi("variant"),flags=gi("flags"),pin_map=gi("pin_map"),
        package_details=gi("package_details"),code_memory_size=gi("code_memory_size"),
        read_buffer_size=gi("read_buffer_size"),write_buffer_size=gi("write_buffer_size"),
        pulse_delay=gi("pulse_delay"),chip_id=gi("chip_id"),chip_info=gi("chip_info"),
        protocol_id=gi("protocol_id"))
    for n in names:
        entries.append((norm(n.strip()),attrs))
xmlmap={k:a for k,a in entries}
overlap=[(k,xmlmap[k],dll[k][1],dll[k][0]) for k in xmlmap if k in dll]
print(f"XML names={len(xmlmap)} DLL names={len(dll)} OVERLAP={len(overlap)}")
def u(raw,off,sz): return int.from_bytes(raw[off:off+sz],"little")
# validate direct fields across overlap
def check(field,fn):
    ok=bad=0; ex=None
    for k,a,raw,proto in overlap:
        if a[field] is None: continue
        if fn(raw)==a[field]: ok+=1
        else:
            bad+=1
            if ex is None and bad<=1: ex=(k,hex(a[field]),hex(fn(raw)))
    print(f"  {field:18} match {ok}/{ok+bad}"+(f"  e.g.{ex}" if ex else ""))
print("direct-field validation:")
check("protocol_id",lambda r:u(r,0,1))
check("code_memory_size",lambda r:u(r,0x38,4))
check("read_buffer_size",lambda r:u(r,0x48,2))
check("write_buffer_size",lambda r:u(r,0x4a,2))
check("pulse_delay",lambda r:u(r,0x58,2))
check("chip_info",lambda r:u(r,0x44,2))
check("chip_id",lambda r:(r[0x5c]<<16)|(r[0x5d]<<8)|r[0x5e])
# variant predictor: for each candidate offset, is variant constant within (proto,offval)?
print("\nvariant predictor search (which desc field determines variant):")
best=[]
for off in range(0,0x74):
    for sz in (1,2,4):
        if off+sz>0x74: continue
        groups=defaultdict(set)
        for k,a,raw,proto in overlap:
            if a["variant"] is None: continue
            groups[(proto,u(raw,off,sz))].add(a["variant"])
        if not groups: continue
        pure=sum(1 for v in groups.values() if len(v)==1)
        frac=pure/len(groups)
        best.append((frac,off,sz,len(groups)))
best.sort(reverse=True)
print("  top (proto,desc[off:sz]) -> variant purity:")
for frac,off,sz,ng in best[:8]:
    print(f"    off={off:#x} sz={sz}: {frac*100:.1f}% pure ({ng} groups)")
