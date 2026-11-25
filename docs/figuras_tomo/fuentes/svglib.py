FILL=dict(hw="#ECEFF1",ctl="#DCEBFA",nav="#E3F2E1",twin="#FFE3C4",model="#EDE3F6",sim="#D8F0EE",white="#FFFFFF")
STROKE=dict(hw="#78909C",ctl="#1E6BB8",nav="#3C8D3F",twin="#E65100",model="#7E57C2",sim="#00897B",white="#757575")
class S:
    def __init__(s,W,H): s.W,s.H,s.o,s.back=W,H,[],[]
    def box(s,x,y,w,h,lines,kind="ctl",bold_first=True,size=24,rx=14,sw=2.5,left=None,sub=None):
        s.o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{FILL[kind]}" stroke="{STROKE[kind]}" stroke-width="{sw}"/>')
        n=len(lines); lh=size*1.3; y0=y+h/2-(n-1)*lh/2+size*0.35
        for i,t in enumerate(lines):
            wt='bold' if (i==0 and bold_first) else 'normal'; sz=size if i==0 else (sub or size-3)
            if left is not None and i>0: s.o.append(f'<text x="{x+left}" y="{y0+i*lh}" font-size="{sz}" font-weight="{wt}" text-anchor="start">{t}</text>')
            else: s.o.append(f'<text x="{x+w/2}" y="{y0+i*lh}" font-size="{sz}" font-weight="{wt}" text-anchor="middle">{t}</text>')
    def panel(s,x,y,w,h,title,sub):
        s.back.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="22" fill="#FAFAFA" stroke="#9E9E9E" stroke-width="2.5"/>')
        s.o.append(f'<text x="{x+w/2}" y="{y+42}" font-size="32" font-weight="bold" text-anchor="middle">{title}</text>')
        s.o.append(f'<text x="{x+w/2}" y="{y+76}" font-size="25" text-anchor="middle" fill="#555">{sub}</text>')
    def arrow(s,pts,color="#424242",w=3,both=False,dash=False):
        d="M"+" L".join(f"{x},{y}" for x,y in pts)
        da=' stroke-dasharray="10,7"' if dash else ''
        mk=f'marker-end="url(#a{color[1:]})"'+(f' marker-start="url(#s{color[1:]})"' if both else '')
        s.o.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{w}"{da} {mk}/>')
        s.colors=getattr(s,'colors',set()); s.colors.add(color)
    def text(s,x,y,t,size=22,anchor="middle",color="#333",italic=False,bold=False):
        st=' font-style="italic"' if italic else ''; b=' font-weight="bold"' if bold else ''
        s.o.append(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" fill="{color}"{st}{b}>{t}</text>')
    def legend(s,x,y,items,cols=3,colw=520):
        for i,(k,t) in enumerate(items):
            xx=x+(i%cols)*colw; yy=y+(i//cols)*46
            s.o.append(f'<rect x="{xx}" y="{yy-24}" width="44" height="30" rx="6" fill="{FILL[k]}" stroke="{STROKE[k]}" stroke-width="2.5"/>')
            s.text(xx+58,yy,t,26,"start","#222")
    def save(s,path):
        defs=''.join(f'<marker id="a{c[1:]}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="18" markerHeight="18" markerUnits="userSpaceOnUse" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker><marker id="s{c[1:]}" viewBox="0 0 10 10" refX="1" refY="5" markerWidth="18" markerHeight="18" markerUnits="userSpaceOnUse" orient="auto"><path d="M10,0 L0,5 L10,10 z" fill="{c}"/></marker>' for c in getattr(s,'colors',set()))
        svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="{s.W}" height="{s.H}" viewBox="0 0 {s.W} {s.H}"><defs>{defs}</defs><style>text{{font-family:"Liberation Sans",Arial,sans-serif;fill:#212121}}</style><rect width="{s.W}" height="{s.H}" fill="#fff"/>{"".join(s.back)}{"".join(s.o)}</svg>'
        open(path,"w").write(svg)
