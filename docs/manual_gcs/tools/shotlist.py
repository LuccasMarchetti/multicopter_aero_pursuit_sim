def T(text, **k): return dict(text=text, **k)
def card(text, n, **k): return dict(text=text, tag='h3', up=1, side='gutter', n=n, top=True, **k)
def g(text, n, tag='*', up=0, **k): return dict(text=text, tag=tag, up=up, side='gutter', n=n, **k)
HIDE = "document.getElementById('scene-pane').style.visibility='hidden'"
SHOW = "document.getElementById('scene-pane').style.visibility=''"
VIEW = lambda v: "document.querySelector('#view-switch button[data-view=\"" + v + "\"]').click()"
FOLLOW = "{const f=document.getElementById('follow'); f.value='x500_depth_0'; f.dispatchEvent(new Event('change'));}"
FRAMES = lambda on: f"{{const c=document.getElementById('frames'); if(c.checked!=={str(on).lower()}) c.click();}}"
VH = 1500
MAXI = "document.querySelector('#scene-pane .pane-bar').dispatchEvent(new MouseEvent('dblclick', {bubbles: true}))"
GUT = (1092, 42, 508, VH - 42)
TOPS = "document.getElementById('panels').scrollTop=0; document.querySelectorAll('#side *').forEach(e=>{if(e.scrollTop) e.scrollTop=0})"
def panel(name, tab, ann, scroll=None, pre=()):
    return dict(name=name, tab=tab, pre=[HIDE, TOPS, *pre], scroll=scroll, ann=ann, clip=GUT, vh=VH)
BADGE = lambda i, n, **k: dict(sel='#badges > *', nth=i, side='bottom', n=n, dist=34, **k)
GREEN = [[76, 175, 80], [79, 195, 247]]
CLICK = lambda t, tag='button': f"{{const b=[...document.querySelectorAll('{tag}')].filter(e=>e.getClientRects().length).find(e=>e.textContent.trim().startsWith({t!r})); b&&b.click();}}"

SHOTS = [
 dict(name='01_visao_geral', tab='Perseguição', pre=[SHOW, VIEW('3d'), FOLLOW, FRAMES(False)],
      actions=[('wheel', (930, 600), -120, 6), ('wait', 1500)],
      ann=[dict(sel='#badges', side='bottom', n=1, dist=70, dx=-400), dict(sel='#layout-menu', side='left', n=2, dist=50),
           dict(sel='#video-wrap', side='right', box=False, n=3, dist=-60, dy=120), dict(sel='#video-stop', side='bottom', n=4),
           dict(sel='#view-switch', side='bottom', n=5), dict(sel='#follow', side='bottom', n=6),
           dict(sel='#clear-trails', side='bottom', n=7, dx=-25), dict(sel='#tabs', side='left', n=8, dy=60),
           dict(sel='#panels', side='left', n=9, box=False)]),
 dict(name='02_cabecalho', pre=[SHOW], clip=(0, 0, 1600, 96),
      ann=[BADGE(0, 1), BADGE(1, 2), BADGE(2, 3), BADGE(3, 4), BADGE(4, 5), BADGE(5, 6), BADGE(6, 7), BADGE(7, 8),
           dict(sel='#layout-btn', side='bottom', n=9, dist=34, dx=-30), dict(sel='#lang-select', side='bottom', n=10, dist=34),
           dict(sel='#toggle-side', side='bottom', n=11, dist=34)]),
 dict(name='03_camera', pre=[SHOW], clip='#video-pane', settle=300, freeze=True,
      ann=[dict(hud=True, side='right', n=1, dist=40), dict(det=True, side='bottom', n=2, dist=60, pad=4),
           dict(sel='#video-stop', side='bottom', n=3, dist=40), dict(sel='#video-full', side='bottom', n=4, dist=40, dx=-10)]),
 dict(name='04_vista3d', tab='Perseguição', pre=[MAXI, SHOW, VIEW('3d'), FOLLOW, FRAMES(True)], clip='#scene-pane',
      actions=[('wheel', (800, 500), -120, 7), ('drag', (800, 500), (850, 470)), ('wait', 1500)],
      post=MAXI, ann=[dict(sel='#view-switch', side='bottom', n=1, dist=40), dict(sel='#follow', side='bottom', n=2, dist=40),
           T('rastros', tag='label', side='bottom', n=3, dist=40), T('TFs', tag='label', side='bottom', n=4, dist=40),
           dict(sel='#clear-trails', side='bottom', n=5, dist=40),
           dict(obj='x500_depth_0', side='left', n=6, size=22), dict(obj='X4', side='right', n=7, size=18),
           dict(obj='estimate', side='top', n=8, size=10), dict(obj='los', side='bottom', n=9, size=6, dist=60),
           dict(obj='path', side='right', n=10, size=8)]),
 dict(name='05_vista_gazebo', pre=[MAXI, SHOW, VIEW('gazebo'), "{const s=document.querySelector('.spectator-bar select'); s.value='follow'; s.dispatchEvent(new Event('change'));}"], clip='#scene-pane', actions=[('wait', 12000)],
      post=MAXI, ann=[dict(sel='#view-switch', side='right', n=1, dist=40), dict(sel='.spectator-bar select', nth=0, side='bottom', n=2, dist=40),
           dict(sel='.spectator-bar select', nth=1, side='bottom', n=3, dist=70), dict(sel='.spectator-bar .muted', side='bottom', n=4, dist=40),
           dict(sel='#spectator', side='left', n=5, box=False, dist=-80, dy=100)]),
 dict(name='06_vista_graficos', tab='Gráficos',
      pre=[MAXI, SHOW, CLICK('Adicionar'), "{const s=[...document.querySelectorAll('select')].filter(e=>e.getClientRects().length).find(e=>[...e.options].some(o=>o.value==='range_error')); if(s){s.value='range_error'; s.dispatchEvent(new Event('change'));}}",
           CLICK('Adicionar'), CLICK('Mostrar gráfico')],
      actions=[('wait', 6000)], clip='#scene-pane',
      post=MAXI, ann=[dict(sel='#view-switch', side='bottom', n=1, dist=40), dict(sel='#plot', side='left', n=2, box=False, dist=-90, dy=-120)]),
 dict(name='07_layout', pre=[SHOW, VIEW('3d'), "document.getElementById('layout-btn').click()"], clip=(1000, 0, 600, 420),
      ann=[dict(sel='#layout-btn', side='left', n=1, dist=50), dict(sel='#layout-pop', side='left', n=2, dist=50)],
      post="document.getElementById('layout-btn').click()"),
 panel('10_simulacao_a', 'Simulação', [card('Execução', 1), g('▶ Iniciar', 2, 'button'), g('■ Parar', 3, 'button', dy=0),
       card('Configuração', 4), g('Controle do perseguidor', 5, 'h4'), g('Entrada e alvo', 6, 'h4')]),
 panel('10_simulacao_b', 'Simulação', [g('Percepção e gravação', 1, 'h4'), g('Gazebo, PX4 e avançado', 2, 'h4'),
       card('Estudos em lote e sintonia', 3), card('Logs', 4), card('Console do PX4', 5)], scroll=T('Percepção e gravação', tag='h4')),
 panel('20_perseguicao_a', 'Perseguição', [card('Estado', 1), g('Modo', 2, '.stat'), g('Rastreador', 3, '.stat'), g('PX4', 4, '.stat'),
       g('Habilitar', 5, 'button'), g('Parar rastreamento', 6, 'button'), g('Frame do alvo', 7, 'label', up=0),
       g('Gimbal', 8, '.stat')]),
 panel('20_perseguicao_b', 'Perseguição', [card('Controlador (ao vivo)', 1), g('controller_type', 2, 'label', up=0), g('target_distance', 3, 'label'),
       g('Centralização do alvo', 4, 'h4'), g('yaw_mode', 5, 'label'), g('vertical_mode', 6, 'label'),
       g('vertical.camera_pitch_source', 7, 'label'), g('vertical.camera_pitch_deg', 8, 'label')],
       scroll=T('Controlador (ao vivo)', tag='h3')),
 panel('20_perseguicao_c', 'Perseguição', [g('Centralização do alvo', 1, 'h4'), g('vertical.camera_pitch_source', 2, 'label'),
       card('Visão (ao vivo)', 3), g('dnn_target', 4, 'label')], scroll=T('Centralização do alvo', tag='h4')),
 panel('20_perseguicao_d', 'Perseguição', [card('Gimbal da câmera (ao vivo)', 1), g('mode', 2, 'label'),
       g('inertia.enabled', 3, 'label'), g('inertia.max_rate_deg_s', 4, 'label'), g('neutral_pitch_deg', 5, 'label'),
       g('attitude_source', 6, 'label')], scroll=T('Gimbal da câmera (ao vivo)', tag='h3')),
 panel('21_controle_a', 'Controle', [card('Lei de controle', 1), card('Supervisor (tempos)', 2), card('Método ativo', 3)]),
 panel('21_controle_b', 'Controle', [card('Demais métodos', 1), card('Comum a todos os métodos', 2), card('Malha interna da PX4', 3)],
       scroll=T('Demais métodos', tag='h3')),
 panel('22_drones', 'Drones', [card('Teleoperação', 1), card('Drones-alvo', 2), card('Mundo (Gazebo)', 3), card('Treinamento', 4)]),
 panel('23_joystick', 'Joystick', [card('Envio', 1), card('Entrada', 2), card('Estado do controle', 3), card('Mapeamento', 4)]),
 panel('24_cenarios_a', 'Cenários', [card('Execução', 1), g('▶ Executar', 2, 'button'), g('■ Parar', 3, 'button'),
       g('⏏ Descarregar', 4, 'button'), card('Arquivos', 5)]),
 panel('24_cenarios_b', 'Cenários', [card('Editor', 1), g('✎ no 3D', 2, 'button')], scroll=T('Editor', tag='h3')),
 panel('25_fisica_a', 'Física e clima', [card('Física aerodinâmica', 1), card('Estado do vento', 2)]),
 panel('25_fisica_b', 'Física e clima', [card('Vento (ao vivo)', 1), g('Frente', 2, 'h4'), g('Rajadas', 3, 'h4'),
       g('Turbulência', 4, 'h4'), g('Zonas', 5, 'h4')], scroll=T('Vento (ao vivo)', tag='h3')),
 panel('25_fisica_c', 'Física e clima', [card('Neblina (ao vivo)', 1)], scroll=T('Neblina (ao vivo)', tag='h3')),
 panel('26_graficos', 'Gráficos', [card('Exibição', 1), card('Séries', 2), card('Adicionar da telemetria', 3), card('Adicionar de um tópico', 4)]),
 panel('27_parametros_a', 'Parâmetros', [g('Nó', 1, 'label'), g('Filtro', 2, 'label'), g('(geral)', 3, 'summary'), g('vertical (', 4, 'summary')]),
 panel('27_parametros_b', 'Parâmetros', [card('Ajustes lembrados', 1), g('Lembrar alterações', 2, 'label'),
       g('/pursuit_controller_node vertical_mode', 3, 'span', up=1), g('Esquecer todos', 4, 'button')],
       scroll=T('Ajustes lembrados', tag='h3')),
 panel('28_video_a', 'Vídeo', [card('Estado do stream', 1), card('Fonte e codec', 2), card('Qualidade', 3)]),
 panel('28_video_b', 'Vídeo', [card('Latência', 1), card('Overlay', 2)], scroll=T('Latência', tag='h3')),
 panel('29_topicos', 'Tópicos', [card('Tópicos', 1), card('Inspeção', 2), card('Publicar', 3), card('Serviços', 4)]),
 panel('30_rede', 'Rede', [card('Qualidade da conexão', 1), card('Acesso remoto', 2), card('Rede local', 3), card('Token de acesso', 4)]),
 panel('31_ajuda', 'Ajuda', [card('Fluxo básico', 1), card('Abas', 2)]),
]
