"""
Poste Inteligente v8 — Análise de Stress Nocturna
Gráficos de comportamento do sistema com 50 postes, via 2500m
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap

# ── Parâmetros do sistema (produção corrigida) ─────────────────────────────
NUM_POSTES       = 50
POSTE_DIST_M     = 50
LIGHT_MIN        = 15       # % — baseline legal EN 13201
LIGHT_MAX        = 100      # %
LIGHT_SAFE_MODE  = 50       # %
FADE_DOWN_MS     = 4000
FADE_UP_RAPIDO   = 300      # ms — ≥ 80 km/h
FADE_UP_MEDIO    = 500      # ms — ≥ 50 km/h
FADE_UP_LENTO    = 800      # ms — ≥ 30 km/h
SPD_FALLBACK_MS  = 500      # ms
TRAFIC_TIMEOUT   = 5000     # ms
UDP_LATENCY_MS   = 5        # ms típico

# ── Paleta nocturna ────────────────────────────────────────────────────────
DARK_BG    = '#0d1117'
AMBER      = '#f0a500'
AMBER_DIM  = '#8b5e00'
BLUE_LIGHT = '#4fc3f7'
RED_WARN   = '#ef5350'
GREEN_OK   = '#66bb6a'
GREY_LINE  = '#30363d'
WHITE      = '#e6edf3'
YELLOW_SPD = '#fff176'

plt.rcParams.update({
    'figure.facecolor': DARK_BG,
    'axes.facecolor':   DARK_BG,
    'axes.edgecolor':   GREY_LINE,
    'axes.labelcolor':  WHITE,
    'xtick.color':      WHITE,
    'ytick.color':      WHITE,
    'text.color':       WHITE,
    'grid.color':       GREY_LINE,
    'grid.alpha':       0.4,
    'font.family':      'monospace',
})


def fade_ms_para_velocidade(vel_kmh):
    if vel_kmh >= 80:  return FADE_UP_RAPIDO
    if vel_kmh >= 50:  return FADE_UP_MEDIO
    if vel_kmh >= 30:  return FADE_UP_LENTO
    return 500


def simular_brightness_poste(t_ms, t_spd_recv, eta_ms, vel_kmh,
                              spd_perdido=False, t_passed=None):
    """
    Simula o nível de brilho de um poste ao longo do tempo.
    t_ms       : array de timestamps
    t_spd_recv : quando o SPD foi recebido (ou None se perdido)
    eta_ms     : ETA enviado no SPD
    vel_kmh    : velocidade do veículo
    spd_perdido: True se o SPD foi perdido (usa fallback)
    t_passed   : quando o veículo passou (PASSED recebido)
    """
    brilho = np.full_like(t_ms, float(LIGHT_MIN))
    fade_up = fade_ms_para_velocidade(vel_kmh)

    if spd_perdido:
        # Fallback: acende ao fim de SPD_FALLBACK_MS após TC_INC
        t_acender = t_spd_recv + SPD_FALLBACK_MS  # t_spd_recv = t_tc_inc aqui
    else:
        # Normal: acende em ETA - fade_up (correcção proposta)
        t_acender = t_spd_recv + eta_ms - fade_up
        t_acender = max(t_acender, t_spd_recv)

    t_full   = t_acender + fade_up
    t_down   = (t_passed + UDP_LATENCY_MS) if t_passed else (t_full + TRAFIC_TIMEOUT)
    t_base   = t_down + FADE_DOWN_MS

    for i, t in enumerate(t_ms):
        if t < t_acender:
            brilho[i] = LIGHT_MIN
        elif t < t_full:
            prog = (t - t_acender) / fade_up
            brilho[i] = LIGHT_MIN + (LIGHT_MAX - LIGHT_MIN) * prog
        elif t < t_down:
            brilho[i] = LIGHT_MAX
        elif t < t_base:
            prog = (t - t_down) / FADE_DOWN_MS
            brilho[i] = LIGHT_MAX - (LIGHT_MAX - LIGHT_MIN) * prog
        else:
            brilho[i] = LIGHT_MIN

    return brilho


# ══════════════════════════════════════════════════════════════════════════════
# FIGURA 1 — Onda de Luz: veículo a percorrer os 50 postes
# ══════════════════════════════════════════════════════════════════════════════
def grafico_onda_luz():
    fig, axes = plt.subplots(3, 1, figsize=(16, 14))
    fig.suptitle('Onda de Luz — Veículo a Percorrer 50 Postes (2500m)',
                 fontsize=14, fontweight='bold', color=WHITE, y=0.98)

    velocidades = [80, 120, 200]
    titulos = ['80 km/h  (via rápida urbana)',
               '120 km/h (limite autoestrada PT)',
               '200 km/h (excesso grave — teste stress)']

    for ax, vel_kmh, titulo in zip(axes, velocidades, titulos):
        vel_ms = vel_kmh / 3.6
        # tempo total para percorrer a cadeia completa + margem
        t_total_ms = int((NUM_POSTES * POSTE_DIST_M / vel_ms) * 1000) + 8000
        t_ms = np.linspace(0, t_total_ms, 3000)

        # Para cada poste, calcular quando o veículo passa
        brilhos = np.full((NUM_POSTES, len(t_ms)), float(LIGHT_MIN))
        fade_up = fade_ms_para_velocidade(vel_kmh)

        for n in range(NUM_POSTES):
            # Quando veículo chega ao poste N (t=0 = entrada na cadeia)
            t_at_n = (n * POSTE_DIST_M / vel_ms) * 1000
            # EVT_LOCAL dispara 1m antes do poste N
            t_local = t_at_n - (1.0 / vel_ms * 1000)
            # TC_INC/SPD enviado → próximo poste N+1
            if n < NUM_POSTES - 1:
                t_spd_recv_n1 = t_local + UDP_LATENCY_MS
                eta_n1 = (POSTE_DIST_M / vel_ms) * 1000
                t_passed_n1 = ((n + 1) * POSTE_DIST_M / vel_ms) * 1000

                brilhos[n + 1] = simular_brightness_poste(
                    t_ms, t_spd_recv_n1, eta_n1, vel_kmh,
                    spd_perdido=False, t_passed=t_passed_n1
                )

            # Poste 0 também acende quando detecção local
            if n == 0:
                t_down = t_at_n + UDP_LATENCY_MS
                t_base = t_down + FADE_DOWN_MS
                for i, t in enumerate(t_ms):
                    if t < t_local:
                        brilhos[0][i] = LIGHT_MIN
                    elif t < t_local + fade_up:
                        prog = (t - t_local) / fade_up
                        brilhos[0][i] = LIGHT_MIN + (LIGHT_MAX - LIGHT_MIN) * prog
                    elif t < t_down + TRAFIC_TIMEOUT:
                        brilhos[0][i] = LIGHT_MAX
                    elif t < t_down + TRAFIC_TIMEOUT + FADE_DOWN_MS:
                        prog = (t - (t_down + TRAFIC_TIMEOUT)) / FADE_DOWN_MS
                        brilhos[0][i] = LIGHT_MAX - (LIGHT_MAX - LIGHT_MIN) * prog
                    else:
                        brilhos[0][i] = LIGHT_MIN

        # Heatmap
        cmap = LinearSegmentedColormap.from_list(
            'amber_night',
            [(0.05, 0.07, 0.09),   # escuro (LIGHT_MIN)
             (0.35, 0.25, 0.0),    # laranja escuro
             (0.94, 0.65, 0.0),    # âmbar
             (1.0,  0.98, 0.8)],   # branco quente (100%)
            N=256
        )

        im = ax.imshow(
            brilhos,
            aspect='auto',
            origin='lower',
            extent=[0, t_total_ms / 1000, 0, NUM_POSTES],
            vmin=LIGHT_MIN, vmax=LIGHT_MAX,
            cmap=cmap,
            interpolation='bilinear'
        )

        # Trajectória do veículo
        t_veiculo = np.linspace(0, (NUM_POSTES * POSTE_DIST_M / vel_ms), 200)
        poste_veiculo = t_veiculo * vel_ms / POSTE_DIST_M
        ax.plot(t_veiculo, poste_veiculo, color=RED_WARN,
                linewidth=2, linestyle='--', label='Veículo', alpha=0.9)

        ax.set_title(titulo, fontsize=10, color=AMBER, pad=4)
        ax.set_xlabel('Tempo (s)', fontsize=9)
        ax.set_ylabel('Poste N.º', fontsize=9)
        ax.set_ylim(0, NUM_POSTES)
        ax.grid(True, axis='x', alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)

        cbar = plt.colorbar(im, ax=ax, pad=0.01)
        cbar.set_label('Brilho (%)', fontsize=8, color=WHITE)
        cbar.ax.yaxis.set_tick_params(color=WHITE)

        # Linha de baseline
        ax.axhline(0, color=GREY_LINE, linewidth=0.5)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    return fig


# ══════════════════════════════════════════════════════════════════════════════
# FIGURA 2 — Perfil de Brilho: SPD normal vs SPD perdido vs sem fix
# ══════════════════════════════════════════════════════════════════════════════
def grafico_perfil_brilho():
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle('Perfil de Brilho por Poste — Comparação de Cenários',
                 fontsize=14, fontweight='bold', color=WHITE)

    velocidades = [80, 120, 160, 200]
    for ax, vel_kmh in zip(axes.flat, velocidades):
        vel_ms = vel_kmh / 3.6
        fade_up = fade_ms_para_velocidade(vel_kmh)
        eta_ms  = int(POSTE_DIST_M / vel_ms * 1000)
        t_passed = eta_ms + int(1 / vel_ms * 1000) + 50

        t_ms = np.linspace(-200, eta_ms + FADE_DOWN_MS + 1000, 2000)

        # Cenário A: SPD entregue com correcção (eta - fade_up)
        b_correcto = simular_brightness_poste(
            t_ms, 0, eta_ms, vel_kmh, spd_perdido=False, t_passed=t_passed)

        # Cenário B: SPD entregue SEM correcção (comportamento actual)
        brilho_bug = np.full_like(t_ms, float(LIGHT_MIN))
        t_acender_bug = eta_ms  # bug: acende no ETA, não ETA - fade
        t_full_bug    = t_acender_bug + fade_up
        t_down_bug    = t_passed + UDP_LATENCY_MS
        t_base_bug    = t_down_bug + FADE_DOWN_MS
        for i, t in enumerate(t_ms):
            if t < t_acender_bug:
                brilho_bug[i] = LIGHT_MIN
            elif t < t_full_bug:
                prog = (t - t_acender_bug) / fade_up
                brilho_bug[i] = LIGHT_MIN + (LIGHT_MAX - LIGHT_MIN) * prog
            elif t < t_down_bug:
                brilho_bug[i] = LIGHT_MAX
            elif t < t_base_bug:
                prog = (t - t_down_bug) / FADE_DOWN_MS
                brilho_bug[i] = LIGHT_MAX - (LIGHT_MAX - LIGHT_MIN) * prog
            else:
                brilho_bug[i] = LIGHT_MIN

        # Cenário C: SPD perdido (fallback 500ms)
        b_fallback = simular_brightness_poste(
            t_ms, 0, eta_ms, vel_kmh, spd_perdido=True, t_passed=t_passed)

        ax.plot(t_ms, b_correcto,  color=GREEN_OK,   lw=2,   label='SPD recebido (com fix ETA)')
        ax.plot(t_ms, brilho_bug,  color=YELLOW_SPD, lw=1.5, label='SPD recebido (sem fix — actual)', linestyle='-.')
        ax.plot(t_ms, b_fallback,  color=BLUE_LIGHT, lw=2,   label=f'SPD perdido (fallback {SPD_FALLBACK_MS}ms)')

        # Linha de chegada do veículo
        ax.axvline(x=eta_ms, color=RED_WARN, linewidth=1.5,
                   linestyle=':', label=f'Veículo chega ({eta_ms}ms)')

        # Nível mínimo legal
        ax.axhline(y=LIGHT_MIN, color=AMBER_DIM, linewidth=1,
                   linestyle='--', alpha=0.7, label=f'Mínimo legal ({LIGHT_MIN}%)')

        # Brilho na chegada (cenário correcto)
        idx_chegada = np.argmin(np.abs(t_ms - eta_ms))
        brilho_na_chegada = b_correcto[idx_chegada]
        brilho_fallback_chegada = b_fallback[idx_chegada]

        ax.set_title(f'{vel_kmh} km/h  |  ETA={eta_ms}ms  |  Fade={fade_up}ms',
                     fontsize=10, color=AMBER)
        ax.set_xlabel('Tempo (ms)', fontsize=8)
        ax.set_ylabel('Brilho (%)', fontsize=8)
        ax.set_ylim(0, 110)
        ax.set_xlim(-200, eta_ms + 6000)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, loc='lower right')

        # Anotação do brilho na chegada
        ax.annotate(f'Na chegada:\n{brilho_na_chegada:.0f}% (fix)\n{brilho_fallback_chegada:.0f}% (fallback)',
                    xy=(eta_ms, brilho_na_chegada),
                    xytext=(eta_ms + 300, 60),
                    fontsize=7, color=WHITE,
                    arrowprops=dict(arrowstyle='->', color=GREY_LINE))

    plt.tight_layout()
    return fig


# ══════════════════════════════════════════════════════════════════════════════
# FIGURA 3 — Velocidade vs Brilho na Chegada
# ══════════════════════════════════════════════════════════════════════════════
def grafico_velocidade_brilho():
    fig, ax = plt.subplots(figsize=(14, 7))
    fig.suptitle('Garantia de Iluminação por Velocidade — 50 Postes × 50m',
                 fontsize=13, fontweight='bold', color=WHITE)

    velocidades = np.arange(30, 310, 5)
    brilho_normal_fix     = []
    brilho_normal_bug     = []
    brilho_fallback       = []

    for vel_kmh in velocidades:
        vel_ms   = vel_kmh / 3.6
        fade_up  = fade_ms_para_velocidade(vel_kmh)
        eta_ms   = POSTE_DIST_M / vel_ms * 1000
        t_arr    = eta_ms + 1 / vel_ms * 1000  # chegada real (1m extra)

        # Com fix ETA (eta - fade_up)
        t_ac = max(0, eta_ms - fade_up)
        if t_arr >= t_ac + fade_up:
            brilho_normal_fix.append(100.0)
        elif t_arr >= t_ac:
            prog = (t_arr - t_ac) / fade_up
            brilho_normal_fix.append(LIGHT_MIN + (LIGHT_MAX - LIGHT_MIN) * prog)
        else:
            brilho_normal_fix.append(float(LIGHT_MIN))

        # Sem fix (bug actual — acende no ETA)
        if t_arr >= eta_ms + fade_up:
            brilho_normal_bug.append(100.0)
        elif t_arr >= eta_ms:
            prog = (t_arr - eta_ms) / fade_up
            brilho_normal_bug.append(LIGHT_MIN + (LIGHT_MAX - LIGHT_MIN) * prog)
        else:
            brilho_normal_bug.append(float(LIGHT_MIN))

        # Fallback (SPD perdido)
        t_fb = SPD_FALLBACK_MS
        if t_arr >= t_fb + fade_up:
            brilho_fallback.append(100.0)
        elif t_arr >= t_fb:
            prog = (t_arr - t_fb) / fade_up
            brilho_fallback.append(LIGHT_MIN + (LIGHT_MAX - LIGHT_MIN) * prog)
        else:
            brilho_fallback.append(float(LIGHT_MIN))

    ax.fill_between(velocidades, 0, LIGHT_MIN,
                    alpha=0.15, color=RED_WARN, label='Abaixo do mínimo legal')
    ax.fill_between(velocidades, LIGHT_MIN, 100,
                    alpha=0.05, color=GREEN_OK)

    ax.plot(velocidades, brilho_normal_fix, color=GREEN_OK,   lw=2.5,
            label='SPD recebido + fix ETA (proposto)')
    ax.plot(velocidades, brilho_normal_bug, color=YELLOW_SPD, lw=2,
            linestyle='-.', label='SPD recebido sem fix (actual)')
    ax.plot(velocidades, brilho_fallback,   color=BLUE_LIGHT, lw=2,
            label=f'SPD perdido — fallback {SPD_FALLBACK_MS}ms')

    # Marcadores de referência
    for v, cor, label in [(120, AMBER, '120 km/h\nlimite PT'),
                          (160, RED_WARN, '160 km/h'),
                          (225, '#ff7043', '225 km/h\nlimite fallback')]:
        ax.axvline(x=v, color=cor, linewidth=1, linestyle=':', alpha=0.7)
        ax.text(v + 2, 5, label, fontsize=7, color=cor, va='bottom')

    ax.axhline(y=100,       color=WHITE,      lw=0.8, linestyle='--', alpha=0.4, label='100%')
    ax.axhline(y=LIGHT_MIN, color=AMBER_DIM,  lw=1.2, linestyle='--', label=f'Mínimo legal {LIGHT_MIN}%')

    ax.set_xlabel('Velocidade do Veículo (km/h)', fontsize=10)
    ax.set_ylabel('Brilho na Chegada ao Poste (%)', fontsize=10)
    ax.set_xlim(30, 305)
    ax.set_ylim(0, 108)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9, loc='lower left')

    ax.text(0.98, 0.97,
            f'POSTE_DIST_M = {POSTE_DIST_M}m\nLIGHT_MIN = {LIGHT_MIN}%\nSPD_FALLBACK = {SPD_FALLBACK_MS}ms',
            transform=ax.transAxes, fontsize=8, color=GREY_LINE,
            ha='right', va='top',
            bbox=dict(boxstyle='round', facecolor=DARK_BG, edgecolor=GREY_LINE))

    plt.tight_layout()
    return fig


# ══════════════════════════════════════════════════════════════════════════════
# FIGURA 4 — Perfil Energético Nocturno (20h–06h)
# ══════════════════════════════════════════════════════════════════════════════
def grafico_perfil_noturno():
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10), sharex=True)
    fig.suptitle('Perfil de Operação Nocturna — 10h (20:00–06:00) · 1000 Veh/h',
                 fontsize=13, fontweight='bold', color=WHITE)

    # Tempo em horas (20h a 06h = 10h)
    horas = np.linspace(0, 10, 5000)
    hora_abs = horas + 20  # hora do dia

    # Perfil de tráfego nocturno (ANSR 2023 — autoestrada PT)
    # 20h-22h: tráfego moderado decrescente
    # 22h-01h: baixo tráfego
    # 01h-04h: tráfego mínimo
    # 04h-06h: tráfego crescente
    def trafego_hora(h_abs):
        if   20 <= h_abs < 21: return 800  - (h_abs - 20) * 200
        elif 21 <= h_abs < 22: return 600  - (h_abs - 21) * 200
        elif 22 <= h_abs < 24: return 400  - (h_abs - 22) * 80
        elif  0 <= h_abs <  2: return 240  - (h_abs - 0)  * 60
        elif  2 <= h_abs <  4: return 120  - (h_abs - 2)  * 20
        elif  4 <= h_abs <  5: return 80   + (h_abs - 4)  * 120
        elif  5 <= h_abs <= 6: return 200  + (h_abs - 5)  * 400
        return 400

    hora_abs_wrapped = np.where(hora_abs >= 24, hora_abs - 24, hora_abs)
    trafego = np.array([trafego_hora(h) for h in hora_abs_wrapped])

    # Brilho médio baseado em tráfego
    # Formula: 5% do tempo a 100% (LIGHT_ON), resto a LIGHT_MIN
    # Com tráfego T veh/hora: fracção de tempo activo ≈ T/3600 × duração_ciclo
    duracao_ciclo_s = (FADE_UP_RAPIDO + TRAFIC_TIMEOUT + FADE_DOWN_MS) / 1000
    fraccao_activo = np.clip(trafego / 3600 * duracao_ciclo_s, 0, 1)
    brilho_medio = LIGHT_MIN + (LIGHT_MAX - LIGHT_MIN) * fraccao_activo

    # Potência média (luminária 100W)
    # LIGHT_MIN=15% → ~15W, LIGHT_MAX=100% → 100W
    potencia_media = 15 + (100 - 15) * fraccao_activo

    # Ax1: Brilho médio e tráfego
    ax1b = ax1.twinx()
    ax1b.fill_between(horas, 0, trafego, color=BLUE_LIGHT, alpha=0.15, label='Tráfego (veh/h)')
    ax1b.plot(horas, trafego, color=BLUE_LIGHT, lw=1, alpha=0.6)
    ax1b.set_ylabel('Tráfego (veh/hora)', fontsize=9, color=BLUE_LIGHT)
    ax1b.tick_params(axis='y', labelcolor=BLUE_LIGHT)
    ax1b.set_ylim(0, 1200)

    ax1.fill_between(horas, LIGHT_MIN, brilho_medio,
                     color=AMBER, alpha=0.4, label='Zona activa (acima baseline)')
    ax1.fill_between(horas, 0, LIGHT_MIN,
                     color=AMBER_DIM, alpha=0.3, label=f'Baseline {LIGHT_MIN}% (EN 13201)')
    ax1.plot(horas, brilho_medio, color=AMBER, lw=2, label='Brilho médio')
    ax1.axhline(y=LIGHT_MIN, color=AMBER_DIM, lw=1.5, linestyle='--')

    ax1.set_ylabel('Brilho médio (%)', fontsize=9)
    ax1.set_ylim(0, 105)
    ax1.grid(True, alpha=0.25)
    ax1.legend(loc='upper right', fontsize=8)
    ax1.set_title('Brilho Médio por Poste vs. Tráfego', fontsize=10, color=AMBER)

    # Ax2: Potência e consumo acumulado
    ax2b = ax2.twinx()
    consumo_kwh = np.cumsum(potencia_media * (horas[1] - horas[0])) / 1000
    ax2b.plot(horas, consumo_kwh, color=RED_WARN, lw=2, label='Consumo acumulado (kWh)')
    ax2b.set_ylabel('Consumo acumulado (kWh)', fontsize=9, color=RED_WARN)
    ax2b.tick_params(axis='y', labelcolor=RED_WARN)

    ax2.fill_between(horas, 0, potencia_media,
                     color=AMBER, alpha=0.35, label='Potência activa (W)')
    ax2.axhline(y=15,  color=AMBER_DIM, lw=1.5, linestyle='--',
                label='Potência baseline 15W')
    ax2.axhline(y=100, color=WHITE,     lw=0.8, linestyle=':', alpha=0.4,
                label='Potência máxima 100W')
    ax2.plot(horas, potencia_media, color=AMBER, lw=2)

    ax2.set_ylabel('Potência média (W)', fontsize=9)
    ax2.set_xlabel('Hora', fontsize=9)
    ax2.set_ylim(0, 115)
    ax2.grid(True, alpha=0.25)
    ax2.legend(loc='upper right', fontsize=8)
    ax2.set_title('Potência Média e Consumo Acumulado por Poste (100W nominal)',
                  fontsize=10, color=AMBER)

    # Etiquetas de hora no eixo X
    horas_ticks = np.arange(0, 11, 1)
    horas_labels = []
    for h in horas_ticks:
        h_abs = (20 + h) % 24
        horas_labels.append(f'{int(h_abs):02d}h')
    ax2.set_xticks(horas_ticks)
    ax2.set_xticklabels(horas_labels, fontsize=9)

    # Zonas de tráfego
    zonas = [(0, 2, 'Tráfego\nModerado'),
             (2, 6, 'Tráfego Baixo / Madrugada'),
             (6, 9, 'Tráfego Reduzido'),
             (9, 10, 'Início\nMatinal')]
    cores_zona = ['#1a3a5c', '#0d1f3c', '#0d1f3c', '#1a3a5c']
    for (t0, t1, lbl), cor in zip(zonas, cores_zona):
        ax1.axvspan(t0, t1, alpha=0.08, color=cor)
        ax1.text((t0 + t1) / 2, 102, lbl, ha='center', va='top',
                 fontsize=7, color=GREY_LINE)

    # Consumo total anotado
    consumo_total = consumo_kwh[-1]
    consumo_sempre_ligado = 100 * 10 / 1000  # 100W × 10h
    poupanca = (1 - consumo_total / consumo_sempre_ligado) * 100
    ax2b.text(0.02, 0.95,
              f'Consumo total: {consumo_total:.2f} kWh/poste/noite\n'
              f'Sempre ligado: {consumo_sempre_ligado:.1f} kWh\n'
              f'Poupança: {poupanca:.0f}%',
              transform=ax2.transAxes, fontsize=9, color=WHITE, va='top',
              bbox=dict(boxstyle='round', facecolor='#1a2535', edgecolor=GREY_LINE, alpha=0.9))

    plt.tight_layout()
    return fig


# ══════════════════════════════════════════════════════════════════════════════
# FIGURA 5 — Carga UDP ao longo da noite
# ══════════════════════════════════════════════════════════════════════════════
def grafico_carga_udp():
    fig, ax = plt.subplots(figsize=(14, 6))
    fig.suptitle('Carga UDP na Rede — 50 Nós · Perfil Nocturno',
                 fontsize=13, fontweight='bold', color=WHITE)

    horas = np.linspace(0, 10, 1000)
    hora_abs = np.where((horas + 20) >= 24, horas + 20 - 24, horas + 20)

    def trafego_hora(h_abs):
        if   20 <= h_abs < 21: return 800  - (h_abs - 20) * 200
        elif 21 <= h_abs < 22: return 600  - (h_abs - 21) * 200
        elif 22 <= h_abs < 24: return 400  - (h_abs - 22) * 80
        elif  0 <= h_abs <  2: return 240  - (h_abs)      * 60
        elif  2 <= h_abs <  4: return 120  - (h_abs - 2)  * 20
        elif  4 <= h_abs <  5: return 80   + (h_abs - 4)  * 120
        elif  5 <= h_abs <= 6: return 200  + (h_abs - 5)  * 400
        return 400

    trafego = np.array([trafego_hora(h) for h in hora_abs])
    taxa = trafego / 3600  # veículos/segundo

    # Pacotes por segundo por tipo (50 nós)
    n = 50
    tc_inc   = taxa * 2 * n   # double-send
    spd      = taxa * 2 * n   # double-send
    passed   = taxa * n
    discover = np.full_like(horas, n * 1.0)  # 1/s por nó

    total = tc_inc + spd + passed + discover

    ax.stackplot(horas,
                 discover, tc_inc, spd, passed,
                 labels=[f'DISCOVER (broadcast, {n}/s fixo)',
                         'TC_INC × 2 (unicast)',
                         'SPD × 2 (unicast)',
                         'PASSED (unicast)'],
                 colors=[AMBER_DIM, BLUE_LIGHT, GREEN_OK, '#ab47bc'],
                 alpha=0.8)

    ax.plot(horas, total, color=WHITE, lw=2, label='Total pkts/s')

    # Capacidade máxima WiFi (conservador)
    ax.axhline(y=5000, color=RED_WARN, lw=1.5, linestyle='--',
               label='Capacidade conservadora 802.11g')

    ax.set_ylabel('Pacotes UDP por segundo', fontsize=10)
    ax.set_xlabel('Hora', fontsize=10)
    ax.set_ylim(0, 300)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=9, loc='upper right')

    horas_ticks  = np.arange(0, 11, 1)
    horas_labels = [f'{int((20 + h) % 24):02d}h' for h in horas_ticks]
    ax.set_xticks(horas_ticks)
    ax.set_xticklabels(horas_labels)

    ax.text(0.5, 0.92, f'Máximo observado: {total.max():.0f} pkts/s  '
            f'({total.max()/5000*100:.1f}% da capacidade WiFi)',
            transform=ax.transAxes, ha='center', fontsize=10,
            color=GREEN_OK,
            bbox=dict(boxstyle='round', facecolor='#1a2535', edgecolor=GREEN_OK))

    plt.tight_layout()
    return fig


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    import os
    out_dir = os.path.join(os.path.dirname(__file__))

    print('A gerar Figura 1 — Onda de Luz...')
    f1 = grafico_onda_luz()
    f1.savefig(os.path.join(out_dir, 'fig1_onda_luz.png'),
               dpi=150, bbox_inches='tight', facecolor=DARK_BG)

    print('A gerar Figura 2 — Perfil de Brilho...')
    f2 = grafico_perfil_brilho()
    f2.savefig(os.path.join(out_dir, 'fig2_perfil_brilho.png'),
               dpi=150, bbox_inches='tight', facecolor=DARK_BG)

    print('A gerar Figura 3 — Velocidade vs Brilho...')
    f3 = grafico_velocidade_brilho()
    f3.savefig(os.path.join(out_dir, 'fig3_velocidade_brilho.png'),
               dpi=150, bbox_inches='tight', facecolor=DARK_BG)

    print('A gerar Figura 4 — Perfil Nocturno...')
    f4 = grafico_perfil_noturno()
    f4.savefig(os.path.join(out_dir, 'fig4_perfil_noturno.png'),
               dpi=150, bbox_inches='tight', facecolor=DARK_BG)

    print('A gerar Figura 5 — Carga UDP...')
    f5 = grafico_carga_udp()
    f5.savefig(os.path.join(out_dir, 'fig5_carga_udp.png'),
               dpi=150, bbox_inches='tight', facecolor=DARK_BG)

    print(f'\nFicheiros gerados em: {out_dir}')
    for nome in ['fig1_onda_luz.png', 'fig2_perfil_brilho.png',
                 'fig3_velocidade_brilho.png', 'fig4_perfil_noturno.png',
                 'fig5_carga_udp.png']:
        path = os.path.join(out_dir, nome)
        if os.path.exists(path):
            print(f'  ✓ {nome}')

    plt.show()
