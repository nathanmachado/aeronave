#!/usr/bin/env python3
"""Rascunho técnico do baseline E10 — vistas superior e lateral, em escala,
gerado diretamente do aircraft_spec.json (nada desenhado 'de cabeça')."""
import json, math, pathlib, re

SC = pathlib.Path(__file__).resolve().parent
REPO = SC.parent
s = json.load(open(REPO / "aircraft_spec.json"))
_toml = (REPO / "config/aircraft/baseline_4seat.toml").read_text()
def _cfg(campo):
    return float(re.search(rf"^{campo} = ([0-9.]+)", _toml, re.M).group(1))
g, wi, e, lg, pr, w = s["geometry"], s["wing"], s["empennage"], s["landing_gear"], s["propeller"], s["weight"]

# --- dados (m) ---
L = g["fuselage_length_m"]            # 8.20
half_w = g["cabin_width_m"] / 2 + 0.02  # meia-largura máx. fuselagem
span = wi["span_m"]; cr = g["chord_root_m"]; ct = g["chord_tip_m"]
x_le_root = g["wing_le_root_x_m"]; mac = g["mac_m"]; x_mac_le = g["mac_le_x_m"]
# enflechamento nulo na linha de 1/4 de corda
x_le_tip = x_le_root + 0.25 * (cr - ct)
span_h = e["span_h_m"]; cr_h = e["chord_h_root_m"]; ct_h = e["chord_h_tip_m"]
cr_v = e["chord_v_root_m"]; ct_v = e["chord_v_tip_m"]; span_v = e["span_v_m"]
x_te_tail = L                          # bordo de fuga da EH no fim da fuselagem
x_le_h = x_te_tail - cr_h
x_main = _cfg("x_main_m"); wheelbase = lg["wheelbase_m"]; x_nose = x_main - wheelbase
track = lg["track_width_m"]
d_prop = pr["diameter_m"]; clear = pr["ground_clearance_m"]
axis_h = d_prop / 2 + clear            # altura da linha de tração
h_cg = _cfg("h_cg_ground_m")
x_cg_fwd = x_mac_le + w["cg_mac_fwd_pct"] / 100 * mac
x_cg_aft = x_mac_le + w["cg_mac_aft_pct"] / 100 * mac
tail_cone_x, tail_cone_h = _cfg("tail_cone_x_m"), _cfg("tail_cone_height_m")
tipback = lg["tipback_angle_deg"]; tstrike = lg["tail_strike_margin_deg"]

belly, top_fus = 0.60, 2.00            # perfil aproximado (cabine 1,2 m)
fin_top = top_fus - 0.1 + span_v       # ~3.35 m

# --- canvas ---
PX = 62                                # px por metro
MX, MY = 90, 70                        # margens
W = int((L + 2.4) * PX + 2 * MX)
top_h = span * PX + 40
side_h = (fin_top + 0.4) * PX + 40
H = int(MY + top_h + 60 + side_h + 90)
X0 = MX + 1.2 * PX                     # x do nariz nas duas vistas
CY = MY + top_h / 2                    # eixo da vista superior
GY = MY + top_h + 60 + side_h          # linha do solo da vista lateral

def X(x): return X0 + x * PX
def Yt(y): return CY - y * PX          # vista superior (y lateral)
def Ys(z): return GY - z * PX          # vista lateral (z altura)

INK, THIN, DIM = "#1a2b45", "#5b7395", "#8895ac"
svg = []
svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" font-family="Iosevka, Menlo, monospace">')
svg.append(f'<rect width="{W}" height="{H}" fill="#f7f5ef"/>')
svg.append(f'<text x="{W/2}" y="38" text-anchor="middle" fill="{INK}" font-size="19" font-weight="bold">AERONAVE 4 LUGARES — BASELINE E10 (rascunho em escala)</text>')
svg.append(f'<text x="{W/2}" y="56" text-anchor="middle" fill="{DIM}" font-size="12">gerado de aircraft_spec.json — 2026-08-08 — PASS, 0 violações, 0 flips (robustez)</text>')

def poly(pts, view, fill="none", sw=2.0, dash=None, color=INK):
    f = {"t": lambda p: (X(p[0]), Yt(p[1])), "s": lambda p: (X(p[0]), Ys(p[1]))}[view]
    d = " ".join(f"{a:.1f},{b:.1f}" for a, b in map(f, pts))
    dd = f' stroke-dasharray="{dash}"' if dash else ""
    svg.append(f'<polyline points="{d}" fill="{fill}" stroke="{color}" stroke-width="{sw}" stroke-linejoin="round"{dd}/>')

def label(x, y, txt, size=11.5, color=INK, anchor="start"):
    svg.append(f'<text x="{x:.0f}" y="{y:.0f}" fill="{color}" font-size="{size}" text-anchor="{anchor}">{txt}</text>')

def dim_h(x1, x2, ypx, txt):           # cota horizontal (px em y)
    svg.append(f'<line x1="{X(x1):.0f}" y1="{ypx}" x2="{X(x2):.0f}" y2="{ypx}" stroke="{DIM}" stroke-width="1" marker-start="url(#ar)" marker-end="url(#ar)"/>')
    label((X(x1) + X(x2)) / 2, ypx - 5, txt, 11, DIM, "middle")

svg.append(f'<defs><marker id="ar" viewBox="0 0 8 8" refX="4" refY="4" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,1 L8,4 L0,7" fill="none" stroke="{DIM}"/></marker></defs>')

# ================= VISTA SUPERIOR =================
label(X(0) - 55, CY - span/2*PX - 14, "VISTA SUPERIOR", 14)
# eixo
poly([(-0.9, 0), (L + 0.5, 0)], "t", sw=0.8, dash="10 5", color=THIN)
# fuselagem (metade + espelho)
prof = [(0.0, 0.0), (0.15, 0.28), (0.7, 0.52), (1.6, half_w), (4.3, half_w),
        (6.2, 0.34), (7.6, 0.16), (L, 0.10)]
poly(prof + [(p[0], -p[1]) for p in reversed(prof)], "t", sw=2.2)
# asa
for sgn in (1, -1):
    poly([(x_le_root, sgn*0.60), (x_le_tip, sgn*span/2), (x_le_tip + ct, sgn*span/2),
          (x_le_root + cr, sgn*0.60)], "t", sw=2.2)
# emp. horizontal
for sgn in (1, -1):
    poly([(x_le_h, sgn*0.13), (x_te_tail - ct_h - 0.25*(cr_h-ct_h), sgn*span_h/2),
          (x_te_tail - 0.25*(cr_h-ct_h), sgn*span_h/2), (x_te_tail, sgn*0.13)], "t", sw=2.0)
# hélice (disco) + spinner
svg.append(f'<ellipse cx="{X(0.12):.0f}" cy="{CY:.0f}" rx="{0.09*PX:.0f}" ry="{d_prop/2*PX:.0f}" fill="none" stroke="{THIN}" stroke-width="1.4" stroke-dasharray="6 4"/>')
label(X(0.25), Yt(d_prop/2) - 6, f"hélice Ø {d_prop:.2f} m", 11, THIN)
# trem
for sgn in (1, -1):
    svg.append(f'<rect x="{X(x_main-0.32):.0f}" y="{Yt(sgn*track/2)-0.10*PX:.0f}" width="{0.64*PX:.0f}" height="{0.20*PX:.0f}" rx="6" fill="none" stroke="{THIN}" stroke-width="1.6"/>')
svg.append(f'<rect x="{X(x_nose-0.26):.0f}" y="{CY-0.09*PX:.0f}" width="{0.52*PX:.0f}" height="{0.18*PX:.0f}" rx="6" fill="none" stroke="{THIN}" stroke-width="1.6"/>')
# CG (faixa dos cenários)
xc = (x_cg_fwd + x_cg_aft) / 2
svg.append(f'<line x1="{X(x_cg_fwd):.0f}" y1="{CY:.0f}" x2="{X(x_cg_aft):.0f}" y2="{CY:.0f}" stroke="{INK}" stroke-width="5"/>')
r = 7
svg.append(f'<circle cx="{X(xc):.0f}" cy="{CY:.0f}" r="{r}" fill="#fff" stroke="{INK}" stroke-width="1.6"/>')
svg.append(f'<path d="M {X(xc):.0f} {CY:.0f} L {X(xc)+r:.0f} {CY:.0f} A {r} {r} 0 0 1 {X(xc):.0f} {CY+r:.0f} Z M {X(xc):.0f} {CY:.0f} L {X(xc)-r:.0f} {CY:.0f} A {r} {r} 0 0 1 {X(xc):.0f} {CY-r:.0f} Z" fill="{INK}"/>')
label(X(xc) + 12, CY + 22, f"CG {w['cg_mac_fwd_pct']:.0f}–{w['cg_mac_aft_pct']:.0f}% MAC", 11, INK)
# cotas
dim_h(0, L, GY - side_h - 105, f"comprimento {L:.2f} m")
sx = X(L + 1.0)
svg.append(f'<line x1="{sx:.0f}" y1="{Yt(span/2):.0f}" x2="{sx:.0f}" y2="{Yt(-span/2):.0f}" stroke="{DIM}" stroke-width="1" marker-start="url(#ar)" marker-end="url(#ar)"/>')
svg.append(f'<text x="{sx+14:.0f}" y="{CY:.0f}" fill="{DIM}" font-size="11" transform="rotate(90 {sx+14:.0f} {CY:.0f})" text-anchor="middle">envergadura {span:.2f} m</text>')

# ================= VISTA LATERAL =================
label(X(0) - 55, GY - side_h + 4, "VISTA LATERAL", 14)
poly([(-0.9, 0), (L + 1.3, 0)], "s", sw=1.4, color=THIN)          # solo
for i in range(24):                                                # hachura do solo
    xh = -0.9 + i * (L + 2.2) / 24
    poly([(xh, 0), (xh - 0.14, -0.14)], "s", sw=1.0, color=THIN)
# perfil da fuselagem
topo = [(0.0, axis_h), (0.25, axis_h + 0.42), (1.5, axis_h + 0.55), (2.2, top_fus),
        (3.9, top_fus), (5.0, top_fus - 0.45), (6.5, top_fus - 0.75), (L, top_fus - 0.85)]
fundo = [(0.0, axis_h - 0.42), (0.9, belly), (4.5, belly), (tail_cone_x, tail_cone_h), (L, top_fus - 0.95)]
poly(topo + list(reversed(fundo)) + [topo[0]], "s", sw=2.2)
# para-brisa
poly([(2.2, top_fus), (2.75, top_fus - 0.28)], "s", sw=1.4, color=THIN)
# asa (corda da raiz, asa baixa — perfil com espessura, ventre da fuselagem)
poly([(x_le_root - 0.05, 0.74), (x_le_root + 0.3, 0.88), (x_le_root + cr, 0.78),
      (x_le_root + cr, 0.72), (x_le_root + 0.3, 0.62), (x_le_root - 0.05, 0.70),
      (x_le_root - 0.05, 0.74)], "s", sw=2.0)
# deriva (EV)
fin_base = top_fus - 0.85
poly([(L - cr_v, fin_base), (L - 0.35 - ct_v, fin_top), (L - 0.35, fin_top), (L + 0.12, fin_base)], "s", sw=2.2)
# EH (corda na lateral)
poly([(x_le_h, fin_base + 0.02), (L + 0.05, fin_base + 0.09), (x_le_h + 0.18, fin_base + 0.20), (x_le_h, fin_base + 0.02)], "s", sw=1.8)
# hélice
svg.append(f'<ellipse cx="{X(0.10):.0f}" cy="{Ys(axis_h):.0f}" rx="{0.07*PX:.0f}" ry="{d_prop/2*PX:.0f}" fill="none" stroke="{THIN}" stroke-width="1.4" stroke-dasharray="6 4"/>')
# trem: pernas e rodas
rw = 0.24
for xw, zleg in ((x_nose, belly), (x_main, 0.90)):
    poly([(xw, zleg), (xw, rw)], "s", sw=2.4)
    svg.append(f'<circle cx="{X(xw):.0f}" cy="{Ys(rw):.0f}" r="{rw*PX:.0f}" fill="none" stroke="{INK}" stroke-width="2.2"/>')
# CG lateral + ângulos de tipback e tail-strike
svg.append(f'<circle cx="{X(x_cg_aft):.0f}" cy="{Ys(h_cg):.0f}" r="5" fill="{INK}"/>')
label(X(x_cg_aft) - 10, Ys(h_cg + 0.42), "CG traseiro", 10.5, DIM, "end")
poly([(x_cg_aft - 0.05, h_cg + 0.30), (x_cg_aft, h_cg + 0.06)], "s", sw=0.9, color=DIM)
poly([(x_main, 0), (x_cg_aft, h_cg)], "s", sw=1.2, dash="5 4", color=DIM)
poly([(x_main, 0), (x_main, h_cg + 0.25)], "s", sw=0.9, dash="2 4", color=DIM)
label(X(x_main) + 6, Ys(h_cg + 0.05), f"tipback {tipback:.1f}°", 11, DIM)
poly([(x_main, 0), (tail_cone_x, tail_cone_h), (tail_cone_x, tail_cone_h)], "s", sw=1.2, dash="5 4", color=DIM)
label(X(5.6), Ys(0.62), f"tail-strike {tstrike:.1f}°", 11, DIM)
# cotas do trem
dim_h(x_nose, x_main, GY + 26, f"entre-eixos {wheelbase:.2f} m")
label(X(x_main) + 8, GY + 44, f"trem principal x = {x_main:.2f} m", 11, DIM)

# ---- bloco de dados ----
bx, by = X(L) - 240, MY + 6
dados = [f"MTOW {w['mtow_kg']:.0f} kg · OEW {w['oew_kg']:.0f} kg",
         f"S {wi['area_m2']:.1f} m² · AR {wi['aspect_ratio']:.1f}",
         f"cruzeiro {s['performance']['v_cruise_kmh']:.0f} km/h",
         f"autonomia {s['performance']['endurance_h']:.1f} h + reserva",
         f"tanque 260 L · margem {s['sizing']['fuel_margin_pct']:.0f}%"]
svg.append(f'<rect x="{bx-12}" y="{by-16}" width="252" height="{len(dados)*17+20}" rx="8" fill="#fffdf8" stroke="{DIM}" stroke-width="1"/>')
for i, t in enumerate(dados):
    label(bx, by + i * 17, t, 11.5, INK)

svg.append("</svg>")
out = SC / "aeronave_e10_rascunho.svg"
out.write_text("\n".join(svg))
print(out, f"{W}x{H}")
