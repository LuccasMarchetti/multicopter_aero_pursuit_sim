"""Annotated GCS screenshots for docs/MANUAL_GCS.md (PT-BR interface, 2x).

Usage: simulation + GCS running (GCS_URL, default http://localhost:8080/), then
    python3 docs/manual_gcs/tools/shots.py [shot names...]
Shots are listed in shotlist.py; annotate.js draws the numbered arrows over DOM elements.
"""
import asyncio, json, sys
from pathlib import Path
from playwright.async_api import async_playwright
import os
OUT = Path(__file__).resolve().parent.parent
URL = os.environ.get('GCS_URL', 'http://localhost:8080/')
ANN = Path(__file__).with_name('annotate.js').read_text()
sys.path.insert(0, str(Path(__file__).parent))
from shotlist import SHOTS
import io
import drawio
from PIL import Image


def quantize(png: bytes) -> bytes:
    """256-colour palette: UI screenshots stay sharp at a third of the size."""
    im = Image.open(io.BytesIO(png)).convert('RGB')
    buf = io.BytesIO()
    im.quantize(256, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE).save(buf, 'PNG', optimize=True)
    return buf.getvalue()
only = set(sys.argv[1:])

async def run_shot(pg, s):
    await pg.set_viewport_size({'width': 1600, 'height': s.get('vh', 1000)})
    await pg.wait_for_timeout(400)
    if s.get('tab'):
        await pg.evaluate("""(t) => { const b=[...document.querySelectorAll('#tabs button')].find(x=>x.textContent.trim().startsWith(t)); b && b.click(); }""", s['tab'])
        await pg.wait_for_timeout(900)
    for js in s.get('pre', []):
        await pg.evaluate(js)
        await pg.wait_for_timeout(300)
    for act in s.get('actions', []):
        kind, *a = act
        if kind == 'wheel':
            await pg.mouse.move(*a[0]); 
            for _ in range(a[2]): await pg.mouse.wheel(0, a[1]); await pg.wait_for_timeout(80)
        elif kind == 'drag':
            await pg.mouse.move(*a[0]); await pg.mouse.down(); await pg.mouse.move(*a[1], steps=12); await pg.mouse.up()
        elif kind == 'wait':
            await pg.wait_for_timeout(a[0])
    if s.get('scroll'):
        await pg.evaluate("""(q) => { const e = window.__find(q); if (e) e.scrollIntoView({block: 'start'}); }""", s['scroll'])
        await pg.wait_for_timeout(500)
    await pg.wait_for_timeout(s.get('settle', 600))
    if s.get('freeze'):   # stop video/overlay redraws so the geometry matches the screenshot
        await pg.evaluate("window.__det = JSON.parse(JSON.stringify(window.gcs.state.detection)); window.__frozen = true; for (const k of ['drawImage','fillRect','strokeRect','fillText','clearRect']) { const o = CanvasRenderingContext2D.prototype[k]; CanvasRenderingContext2D.prototype['__'+k] = o; CanvasRenderingContext2D.prototype[k] = function(...a){ if (!window.__frozen) return o.apply(this, a); }; }")
        await pg.wait_for_timeout(300)
    res = await pg.evaluate('(items) => window.__annotate(items, false)', s.get('ann', []))
    if res['missing']: print(f"  {s['name']}: callouts not found {res['missing']}")
    clip = None
    if s.get('clip'):
        c = s['clip']
        if isinstance(c, str):
            r = await pg.evaluate("(sel) => document.querySelector(sel).getBoundingClientRect().toJSON()", c)
            clip = {'x': r['x'], 'y': r['y'], 'width': r['width'], 'height': r['height']}
        else:
            clip = dict(zip(('x', 'y', 'width', 'height'), c))
    if clip is None:
        vp = pg.viewport_size
        clip = {'x': 0, 'y': 0, 'width': vp['width'], 'height': vp['height']}
    png = await pg.screenshot(clip=clip)
    png = quantize(png)
    shapes = [dict(sh, rect=[sh['rect'][0] - clip['x'], sh['rect'][1] - clip['y'], sh['rect'][2] - clip['x'], sh['rect'][3] - clip['y']] if sh['rect'] else None,
                   badge=[sh['badge'][0] - clip['x'], sh['badge'][1] - clip['y']],
                   arrow=[v - (clip['x'] if i % 2 == 0 else clip['y']) for i, v in enumerate(sh['arrow'])] if sh['arrow'] else None)
              for sh in res['shapes']]
    drawio.write(OUT / f"{s['name']}.drawio.svg", png, clip['width'], clip['height'], shapes)
    if s.get('post'): await pg.evaluate(s['post'])
    if s.get('freeze'):
        await pg.evaluate("window.__frozen = false")
    print('ok', s['name'])

async def main():
    async with async_playwright() as p:
        b = await p.chromium.launch(channel='chrome', headless=True, args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader'])
        pg = await b.new_page(viewport={'width': 1600, 'height': 1000}, device_scale_factor=2)
        await pg.add_init_script("localStorage.setItem('gcs.lang','pt-br'); localStorage.setItem('gcs.view','3d');")
        await pg.add_init_script(ANN)
        await pg.goto(URL, wait_until='networkidle'); await pg.wait_for_timeout(4000)
        for s in SHOTS:
            if only and s['name'] not in only: continue
            try: await run_shot(pg, s)
            except Exception as e: print('FAIL', s['name'], e)
        await b.close()
asyncio.run(main())
