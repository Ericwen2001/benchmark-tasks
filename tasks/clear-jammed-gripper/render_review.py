"""Three native images side by side, no bottom information panel."""
import argparse,json,subprocess
from pathlib import Path
from PIL import Image,ImageDraw
p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('destination');a=p.parse_args();src=Path(a.source);dst=Path(a.destination);dst.mkdir(parents=True,exist_ok=True)
rows=json.loads((src/'trace.json').read_text())
proc=subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','960x200','-r','4','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(dst/'prototype.mp4')],stdin=subprocess.PIPE)
for i,row in enumerate(rows):
 im=Image.new('RGB',(960,200),'#162331');draw=ImageDraw.Draw(im)
 for j,(cam,title) in enumerate([('top_cam','Top'),('left_cam','Left wrist'),('right_cam','Right wrist')]):
  draw.text((j*320+8,5),title,fill='white');im.paste(Image.open(src/f'{cam}_{row["step"]:04d}.png').resize((320,176)),(320*j,24))
 proc.stdin.write(im.tobytes())
 if i in (0,len(rows)-1):im.save(dst/('initial.png' if i==0 else 'final.png'))
proc.stdin.close();assert proc.wait()==0
