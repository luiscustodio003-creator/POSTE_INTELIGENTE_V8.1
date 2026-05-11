/**
 * @file web_html_pages.c
 * @brief Páginas HTML Embebidas (Minificadas)
 * 
 * NOTA: HTML minificado para economizar Flash/RAM
 * Tamanho total: ~8KB comprimido
 * 
 * @author Luis Custodio | Tiago Moreno
 * @date 2026-05-09
 */

#include <stddef.h>

// ============================================================================
// DASHBOARD PRINCIPAL (Vista de Linha)
// ============================================================================

static const char html_dashboard[] = 
"<!DOCTYPE html>"
"<html lang=pt>"
"<head>"
"<meta charset=UTF-8>"
"<meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Poste v8 - Dashboard</title>"
"<style>"
"*{margin:0;padding:0;box-sizing:border-box}"
"body{font:14px/1.5 Arial,sans-serif;background:#1a1a2e;color:#eee;padding:10px}"
".hdr{background:linear-gradient(135deg,#667eea,#764ba2);padding:20px;border-radius:8px;margin-bottom:16px}"
".hdr h1{font-size:22px;margin-bottom:6px}"
".card{background:#16213e;padding:14px;margin:10px 0;border-radius:6px;border-left:3px solid #667eea}"
".metric{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #0f3460}"
".metric:last-child{border-bottom:none}"
".val{color:#667eea;font-weight:bold}"
".badge{display:inline-block;padding:3px 8px;border-radius:10px;font-size:11px;margin-left:6px}"
".ok{background:#27ae60;color:#fff}"
".err{background:#e74c3c;color:#fff}"
".mst{background:#f39c12;color:#000}"
"button{background:#667eea;color:#fff;border:none;padding:10px 16px;border-radius:5px;cursor:pointer;margin:4px;font-size:13px}"
".poste{background:#0f3460;padding:12px;margin:6px 0;border-radius:6px;cursor:pointer}"
".poste:hover{background:#1a4d7a}"
"</style>"
"</head>"
"<body>"
"<div class=hdr>"
"<h1>🚦 Poste Inteligente v8</h1>"
"<div style='font-size:12px;opacity:0.9'>Sistema de Iluminação Adaptativa</div>"
"</div>"

"<div class=card>"
"<h3 style='margin-bottom:10px;color:#667eea'>📊 Estatísticas Globais</h3>"
"<div class=metric><span>Total Veículos</span><span class=val id=tv>0</span></div>"
"<div class=metric><span>Consumo Hoje</span><span class=val id=ec>0.0 kWh</span></div>"
"<div class=metric><span>Economia</span><span class=val id=sv style='color:#27ae60'>0%</span></div>"
"</div>"

"<div class=card>"
"<h3 style='margin-bottom:10px;color:#667eea'>📍 Postes Activos</h3>"
"<div id=lista></div>"
"</div>"

"<button onclick=load()>🔄 Actualizar</button>"
"<button onclick='location.reload()'>🏠 Reset</button>"

"<script>"
"async function load(){"
"try{"
"const r=await fetch('/api/line');"
"const d=await r.json();"
"document.getElementById('tv').textContent=d.stats.total_vehicles;"
"document.getElementById('ec').textContent=d.stats.total_energy.toFixed(2)+' kWh';"
"document.getElementById('sv').textContent=d.stats.energy_saved_percent.toFixed(1)+'%';"
"let html='';"
"d.postes.forEach(p=>{"
"const st=p.is_online?'ok':'err';"
"const role=p.role=='MASTER'?'<span class=\"badge mst\">MASTER</span>':'';"
"html+=`<div class=poste onclick=\"location.href='/poste/${p.position}'\"><div style='display:flex;justify-content:space-between'><div><strong>Poste #${p.position}</strong> ${role}</div><div><span class=\"badge ${st}\">${p.is_online?'🟢 ON':'🔴 OFF'}</span></div></div><div style='font-size:12px;color:#aaa;margin-top:4px'>${p.ip} • ${p.state} • ${p.duty_cycle}%</div></div>`;"
"});"
"document.getElementById('lista').innerHTML=html;"
"}catch(e){alert('Erro: '+e)}"
"}"
"load();"
"</script>"
"</body>"
"</html>";

// ============================================================================
// PÁGINA DE DETALHES DE POSTE
// ============================================================================

static const char html_poste_detail[] = 
"<!DOCTYPE html>"
"<html lang=pt>"
"<head>"
"<meta charset=UTF-8>"
"<meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Poste - Detalhes</title>"
"<style>"
"*{margin:0;padding:0;box-sizing:border-box}"
"body{font:14px/1.5 Arial,sans-serif;background:#1a1a2e;color:#eee;padding:10px}"
".hdr{background:linear-gradient(135deg,#667eea,#764ba2);padding:20px;border-radius:8px;margin-bottom:16px}"
".hdr h1{font-size:22px}"
".card{background:#16213e;padding:14px;margin:10px 0;border-radius:6px;border-left:3px solid #667eea}"
".metric{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #0f3460}"
".metric:last-child{border-bottom:none}"
".val{color:#667eea;font-weight:bold}"
"button{background:#667eea;color:#fff;border:none;padding:10px 16px;border-radius:5px;cursor:pointer;margin:4px}"
".gauge{width:100px;height:100px;margin:10px auto;position:relative}"
".gauge svg{transform:rotate(-90deg)}"
".gauge-txt{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);font-size:20px;font-weight:bold}"
"</style>"
"</head>"
"<body>"
"<div class=hdr>"
"<a href='/dashboard' style='color:#fff;text-decoration:none'>← Voltar</a>"
"<h1>🚦 Poste #<span id=pos>?</span></h1>"
"<div style='font-size:12px;margin-top:4px'>IP: <span id=ip>-.-.-.-</span></div>"
"</div>"

"<div class=card>"
"<h3 style='margin-bottom:8px;color:#667eea'>📡 Estado</h3>"
"<div class=metric><span>Modo</span><span class=val id=st>-</span></div>"
"<div class=metric><span>T / Tc</span><span class=val><span id=T>0</span>/<span id=Tc>0</span></span></div>"
"<div class=metric><span>Duty</span><span class=val id=duty>0%</span></div>"
"<div class=metric><span>Papel</span><span class=val id=role>-</span></div>"
"</div>"

"<div class=card style='text-align:center'>"
"<h3 style='margin-bottom:10px;color:#f39c12'>💡 Intensidade</h3>"
"<div class=gauge>"
"<svg viewBox='0 0 100 100' width=100 height=100>"
"<circle cx=50 cy=50 r=40 fill=none stroke=#0f3460 stroke-width=8/>"
"<circle id=circ cx=50 cy=50 r=40 fill=none stroke=#f39c12 stroke-width=8 stroke-dasharray=251 stroke-dashoffset=251/>"
"</svg>"
"<div class=gauge-txt id=gtxt>0%</div>"
"</div>"
"</div>"

"<div class=card>"
"<h3 style='margin-bottom:8px;color:#27ae60'>⏱️ Tempo (hoje)</h3>"
"<div class=metric><span>SAVE (10%)</span><span class=val id=ts>0h</span></div>"
"<div class=metric><span>MIN (50%)</span><span class=val id=tm>0h</span></div>"
"<div class=metric><span>ON (100%)</span><span class=val id=to>0h</span></div>"
"</div>"

"<div class=card>"
"<h3 style='margin-bottom:8px;color:#3498db'>⚡ Energia</h3>"
"<div class=metric><span>Consumo</span><span class=val id=ekwh>0.0 kWh</span></div>"
"<div class=metric><span>Poupança</span><span class=val id=esv style='color:#27ae60'>0%</span></div>"
"</div>"

"<div class=card>"
"<h3 style='margin-bottom:8px;color:#9b59b6'>🔗 Vizinhos</h3>"
"<div id=nb>-</div>"
"</div>"

"<button onclick=load()>🔄 Actualizar</button>"
"<button onclick=\"location.href='/dashboard'\">🏠 Dashboard</button>"

"<script>"
"const pos=location.hash.slice(1)||location.pathname.split('/').pop()||'0';"
"const API='/api/poste/'+pos;"
"function fmt(s){const h=Math.floor(s/3600);const m=Math.floor(s%3600/60);return h+'h'+m+'m'}"
"async function load(){"
"try{"
"const r=await fetch(API);"
"const d=await r.json();"
"document.getElementById('pos').textContent=d.position;"
"document.getElementById('ip').textContent=d.ip;"
"document.getElementById('st').textContent=d.state;"
"document.getElementById('role').textContent=d.role;"
"document.getElementById('T').textContent=d.T;"
"document.getElementById('Tc').textContent=d.Tc;"
"document.getElementById('duty').textContent=d.duty_cycle+'%';"
"const circ=document.getElementById('circ');"
"const off=251-(d.duty_cycle/100*251);"
"circ.style.strokeDashoffset=off;"
"document.getElementById('gtxt').textContent=d.duty_cycle+'%';"
"const ts=d.time_stats;"
"document.getElementById('ts').textContent=fmt(ts.save_seconds);"
"document.getElementById('tm').textContent=fmt(ts.min_seconds);"
"document.getElementById('to').textContent=fmt(ts.on_seconds);"
"const e=d.energy;"
"document.getElementById('ekwh').textContent=e.consumed_kwh.toFixed(2)+' kWh';"
"document.getElementById('esv').textContent=e.saved_percent.toFixed(1)+'%';"
"const nb=d.neighbors.map(n=>`#${n.position} ${n.ip} ${n.is_alive?'🟢':'🔴'}`).join('<br>');"
"document.getElementById('nb').innerHTML=nb||'Nenhum';"
"}catch(e){alert('Erro: '+e)}"
"}"
"load();"
"</script>"
"</body>"
"</html>";

// ============================================================================
// FUNÇÕES EXPORTADAS
// ============================================================================

const char* get_dashboard_html(void) {
    return html_dashboard;
}

const char* get_poste_detail_html(void) {
    return html_poste_detail;
}
