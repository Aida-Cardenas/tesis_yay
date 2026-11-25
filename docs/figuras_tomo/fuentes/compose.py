from PIL import Image, ImageDraw, ImageFont, ImageChops
import numpy as np
F="/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"
FB="/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"
def trim(im,pad=30):
    bg=Image.new(im.mode,im.size,(255,255,255)); diff=ImageChops.difference(im.convert('RGB'),bg)
    bb=diff.getbbox(); x0,y0,x1,y1=bb
    return im.crop((max(0,x0-pad),max(0,y0-pad),min(im.width,x1+pad),min(im.height,y1+pad)))
def fit(im,w,h):
    im=trim(im.convert('RGB')); s=min(w/im.width,h/im.height); im=im.resize((int(im.width*s),int(im.height*s)),Image.LANCZOS)
    c=Image.new('RGB',(w,h),'white'); c.paste(im,((w-im.width)//2,(h-im.height)//2)); return c
cw,ch=1100,900
panels=[('iso','(a) Vista isométrica'),('front','(b) Vista frontal'),('side','(c) Vista lateral'),('top','(d) Vista superior'),('plates','(e) Placas Base, Medio y Tope')]
lab_h=90
W=Image.new('RGB',(cw*3,(ch+lab_h)*2),'white'); d=ImageDraw.Draw(W); f=ImageFont.truetype(F,46)
for i,(v,t) in enumerate(panels):
    im=fit(Image.open(f'out_{v}.png'),cw-60,ch-40)
    if v=='plates':
        a=np.array(im); orange=(a[:,:,0]>150)&(a[:,:,1]>60)&(a[:,:,1]<140)&(a[:,:,2]<90)
        rows=np.where(orange.any(1))[0]; gaps=np.where(np.diff(rows)>15)[0]
        starts=[rows[0]]+[rows[g+1] for g in gaps]; ends=[rows[g] for g in gaps]+[rows[-1]]
        dd=ImageDraw.Draw(im); fb=ImageFont.truetype(FB,44)
        for (s,e),name in zip(zip(starts,ends),['Tope','Medio','Base']):
            cols=np.where(orange[s:e+1].any(0))[0]; x=cols.max()+25; y=(s+e)//2-24
            dd.text((x,y),name,font=fb,fill=(40,40,40))
    col=(i%3) if i<3 else (0.5 if v=='top' else 1.5); x=int(col*cw)+30; y=(i//3)*(ch+lab_h)+20
    W.paste(im,(x,y))
    tw=d.textlength(t,font=f); d.text((col*cw+(cw-tw)/2,(i//3)*(ch+lab_h)+ch+10),t,font=f,fill=(30,30,30))
W.save('figura_modelo_cad.png',dpi=(300,300)); print(W.size)
