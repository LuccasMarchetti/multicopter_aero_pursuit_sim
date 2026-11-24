"""Writes an editable .drawio.svg: a screenshot (locked background) plus annotation shapes.

The file is a plain SVG (rendered by GitHub, VS Code and browsers) whose `content` attribute
holds the draw.io model. Opening it in draw.io (desktop, app.diagrams.net or the VS Code
"Draw.io Integration" extension) edits boxes, arrows and numbers as separate shapes; saving
rewrites both the model and the SVG preview.

Shapes (CSS px, relative to the screenshot): {n, rect: [x0, y0, x1, y1] | None,
badge: [x, y], arrow: [x1, y1, x2, y2] | None}.
"""
import base64
from html import escape

RED = '#FF3D00'
BADGE_R = 14


def _attr(s: str) -> str:
    return escape(s, quote=True)


def model_xml(png_b64: str, width: float, height: float, shapes: list) -> str:
    cells = [
        '<mxCell id="0"/>',
        '<mxCell id="1" parent="0"/>',
        # Screenshot: own layer, locked so that dragging edits only the annotations.
        '<mxCell id="bg" value="Screenshot" parent="0"/>',
        f'<mxCell id="img" value="" style="shape=image;imageAspect=0;image=data:image/png,{png_b64};'
        f'locked=1;movable=0;resizable=0;editable=0;deletable=0;" vertex="1" parent="bg">'
        f'<mxGeometry x="0" y="0" width="{width:.1f}" height="{height:.1f}" as="geometry"/></mxCell>',
        '<mxCell id="ann" value="Anotações" parent="0"/>',
    ]
    for s in shapes:
        n = s['n']
        rid = f'box{n}'
        if s.get('rect'):
            x0, y0, x1, y1 = s['rect']
            cells.append(
                f'<mxCell id="{rid}" value="" style="rounded=1;arcSize=6;absoluteArcSize=1;whiteSpace=wrap;'
                f'fillColor={RED};opacity=100;fillOpacity=8;strokeColor={RED};strokeWidth=2.5;shadow=1;" '
                f'vertex="1" parent="ann"><mxGeometry x="{x0:.1f}" y="{y0:.1f}" width="{x1 - x0:.1f}" '
                f'height="{y1 - y0:.1f}" as="geometry"/></mxCell>')
        bx, by = s['badge']
        bid = f'n{n}'
        cells.append(
            f'<mxCell id="{bid}" value="{n}" style="ellipse;whiteSpace=wrap;html=1;aspect=fixed;'
            f'fillColor={RED};strokeColor=#FFFFFF;strokeWidth=2;fontColor=#FFFFFF;fontStyle=1;fontSize=15;'
            f'shadow=1;" vertex="1" parent="ann"><mxGeometry x="{bx - BADGE_R:.1f}" y="{by - BADGE_R:.1f}" '
            f'width="{2 * BADGE_R}" height="{2 * BADGE_R}" as="geometry"/></mxCell>')
        if s.get('arrow'):
            _, _, ax, ay = s['arrow']
            style = (f'endArrow=block;endFill=1;html=1;strokeColor={RED};strokeWidth=3;'
                     'edgeStyle=none;rounded=0;')
            if s.get('rect'):
                x0, y0, x1, y1 = s['rect']
                ex = min(max((ax - x0) / max(x1 - x0, 1e-6), 0.0), 1.0)
                ey = min(max((ay - y0) / max(y1 - y0, 1e-6), 0.0), 1.0)
                style += f'entryX={ex:.3f};entryY={ey:.3f};entryDx=0;entryDy=0;entryPerimeter=0;'
                cells.append(
                    f'<mxCell id="a{n}" value="" style="{style}" edge="1" parent="ann" source="{bid}" '
                    f'target="{rid}"><mxGeometry relative="1" as="geometry"/></mxCell>')
            else:
                cells.append(
                    f'<mxCell id="a{n}" value="" style="{style}" edge="1" parent="ann" source="{bid}">'
                    f'<mxGeometry relative="1" as="geometry"><mxPoint x="{ax:.1f}" y="{ay:.1f}" '
                    f'as="targetPoint"/></mxGeometry></mxCell>')
    graph = (f'<mxGraphModel dx="{width:.0f}" dy="{height:.0f}" grid="0" gridSize="10" guides="1" '
             f'tooltips="1" connect="1" arrows="1" fold="1" page="0" pageScale="1" '
             f'pageWidth="{width:.0f}" pageHeight="{height:.0f}" math="0" shadow="0"><root>'
             + ''.join(cells) + '</root></mxGraphModel>')
    return (f'<mxfile host="aero_gcs_manual" type="device"><diagram id="manual" name="Página-1">'
            f'{graph}</diagram></mxfile>')


def preview_svg(png_b64: str, width: float, height: float, shapes: list) -> str:
    """SVG drawing equivalent to the model (what viewers show before any edit in draw.io)."""
    out = [
        '<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" '
        f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{RED}"/></marker>'
        '<filter id="sh" x="-20%" y="-20%" width="140%" height="140%"><feDropShadow dx="0" dy="1" '
        'stdDeviation="1.5" flood-color="#000" flood-opacity="0.8"/></filter></defs>',
        f'<image x="0" y="0" width="{width:.1f}" height="{height:.1f}" preserveAspectRatio="none" '
        f'href="data:image/png;base64,{png_b64}"/>',
    ]
    for s in shapes:
        if s.get('rect'):
            x0, y0, x1, y1 = s['rect']
            out.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{x1 - x0:.1f}" height="{y1 - y0:.1f}" rx="6" '
                       f'fill="{RED}" fill-opacity="0.08" stroke="{RED}" stroke-width="2.5" filter="url(#sh)"/>')
    for s in shapes:
        bx, by = s['badge']
        if s.get('arrow'):
            _, _, ax, ay = s['arrow']
            d = max(((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5, 1e-6)
            ux, uy = (ax - bx) / d, (ay - by) / d
            out.append(f'<line x1="{bx + ux * (BADGE_R + 1):.1f}" y1="{by + uy * (BADGE_R + 1):.1f}" '
                       f'x2="{ax - ux * 2:.1f}" y2="{ay - uy * 2:.1f}" stroke="{RED}" stroke-width="3" '
                       'stroke-linecap="round" marker-end="url(#ah)"/>')
        out.append(f'<circle cx="{bx:.1f}" cy="{by:.1f}" r="{BADGE_R}" fill="{RED}" stroke="#fff" '
                   f'stroke-width="2" filter="url(#sh)"/>')
        out.append(f'<text x="{bx:.1f}" y="{by + 5:.1f}" text-anchor="middle" font-size="15" '
                   f'font-weight="700" font-family="Helvetica,Arial,sans-serif" fill="#fff">{s["n"]}</text>')
    return ''.join(out)


def write(path, png_bytes: bytes, width: float, height: float, shapes: list) -> None:
    b64 = base64.b64encode(png_bytes).decode()
    content = model_xml(b64, width, height, shapes)
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
           f'version="1.1" width="{width:.0f}px" height="{height:.0f}px" '
           f'viewBox="0 0 {width:.0f} {height:.0f}" content="{_attr(content)}">'
           + preview_svg(b64, width, height, shapes) + '</svg>\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(svg)
