import re

file_path = "/home/mell/Downloads/autonomous-multi-drone-swarm-px4/drone-command-center.html"
with open(file_path, "r") as f:
    content = f.read()

# 1. Restore left CSS
left_css = """    .left {
      width: 240px;
      background: var(--bg1);
      border-right: 1px solid var(--border);
      display: flex;
      flex-direction: column;
      flex-shrink: 0;
      overflow: hidden
    }

    .ph {
      font-family: var(--mono);
      font-size: 9px;
      font-weight: 600;
      letter-spacing: 2px;
      color: var(--muted);
      padding: 10px 14px 8px;
      border-bottom: 1px solid var(--border);
      display: flex;
      justify-content: space-between;
      align-items: center;
      text-transform: uppercase
    }

    .ph .cnt {
      color: var(--accent);
      font-size: 14px
    }

    .dlist {
      flex: 1;
      overflow-y: auto;
      padding: 6px;
      display: flex;
      flex-direction: column;
      gap: 4px
    }

    .dlist::-webkit-scrollbar {
      width: 3px
    }

    .dlist::-webkit-scrollbar-thumb {
      background: var(--border2)
    }

    .dc {
      background: var(--bg2);
      border: 1px solid var(--border);
      padding: 10px 11px;
      cursor: pointer;
      transition: all .2s;
      border-radius: 6px;
      position: relative;
      overflow: hidden
    }

    .dc::before {
      content: '';
      position: absolute;
      left: 0;
      top: 0;
      bottom: 0;
      width: 3px;
      background: var(--ok);
      border-radius: 3px 0 0 3px;
      transition: background .3s
    }

    .dc.warn::before {
      background: var(--warn)
    }

    .dc.crit::before {
      background: var(--crit);
      animation: blink .8s infinite
    }

    @keyframes blink {

      0%,
      100% {
        opacity: 1
      }

      50% {
        opacity: .3
      }
    }

    .dc:hover,
    .dc.sel {
      border-color: var(--accent2);
      background: var(--bg3)
    }

    .dc.sel {
      border-color: var(--accent)
    }

    .dch {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 7px
    }

    .did {
      font-family: var(--mono);
      font-size: 12px;
      font-weight: 600;
      color: var(--accent)
    }

    .dst {
      font-size: 9px;
      padding: 2px 7px;
      border-radius: 10px;
      border: 1px solid var(--ok);
      color: var(--ok);
      font-family: var(--mono)
    }

    .dst.warn {
      border-color: var(--warn);
      color: var(--warn)
    }

    .dst.crit {
      border-color: var(--crit);
      color: var(--crit)
    }

    .mg {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 3px;
      margin-bottom: 7px
    }

    .met {
      background: var(--bg);
      border-radius: 4px;
      padding: 5px 7px
    }

    .ml {
      font-size: 8px;
      color: var(--muted);
      letter-spacing: .5px;
      text-transform: uppercase;
      margin-bottom: 2px
    }

    .mv {
      font-size: 13px;
      font-family: var(--mono);
      color: var(--text);
      line-height: 1
    }

    .mv .u {
      font-size: 9px;
      color: var(--text2)
    }

    .bb {
      height: 3px;
      background: var(--bg);
      border-radius: 2px;
      overflow: hidden;
      margin-top: 5px
    }

    .bf {
      height: 100%;
      border-radius: 2px;
      transition: width .5s, background .5s
    }

    .blbl {
      display: flex;
      justify-content: space-between;
      font-size: 8px;
      color: var(--text2);
      font-family: var(--mono);
      margin-top: 4px
    }

    .pos {
      font-size: 8px;
      color: var(--muted);
      font-family: var(--mono);
      letter-spacing: .3px;
      margin-top: 3px
    }

    .map-wrap {"""
content = content.replace("    .map-wrap {", left_css)

# 2. Restore left HTML
left_html = """  <div class="main">
    <div class="left">
      <div class="ph">FLEET <div class="cnt" id="cntDisp">0</div>
      </div>
      <div class="dlist" id="droneList"></div>
    </div>
    <div class="map-wrap">"""
content = content.replace("""  <div class="main">\n    <div class="map-wrap">""", left_html)

# 3. Restore JS
js_render_card = """    function renderCard(id) {
      const d = drones[id];
      let card = document.getElementById(`dc-${id}`);
      const isNew = !card;
      if (isNew) {
        card = document.createElement('div');
        card.className = 'dc'; card.id = `dc-${id}`;
        card.onclick = () => selectDrone(id);
        document.getElementById('droneList').appendChild(card);
      }
      card.className = `dc ${d.status}${selId === id ? ' sel' : ''}`;
      const bc = d.battery < 20 ? 'var(--crit)' : d.battery < 40 ? 'var(--warn)' : 'var(--ok)';
      card.innerHTML = `
    <div class="dch">
      <div class="did">UAV-${String(id).padStart(2, '0')}</div>
      <div class="dst ${d.status}">${d.status.toUpperCase()}</div>
    </div>
    <div class="mg">
      <div class="met"><div class="ml">ALT</div><div class="mv">${d.alt.toFixed(1)}<span class="u"> m</span></div></div>
      <div class="met"><div class="ml">SPD</div><div class="mv">${d.speed.toFixed(1)}<span class="u"> m/s</span></div></div>
      <div class="met"><div class="ml">HDG</div><div class="mv">${d.heading.toFixed(0)}<span class="u"> °</span></div></div>
      <div class="met"><div class="ml">BAT</div><div class="mv" style="color:${bc}">${d.battery.toFixed(0)}<span class="u"> %</span></div></div>
    </div>
    <div class="blbl"><span>CHARGE</span><span style="color:${bc}">${d.battery.toFixed(1)}%</span></div>
    <div class="bb"><div class="bf" style="width:${d.battery}%;background:${bc}"></div></div>
    <div class="pos">${d.lat.toFixed(5)}°N &nbsp;${d.lon.toFixed(5)}°E</div>`;
      document.getElementById('cntDisp').textContent = Object.keys(drones).length;
    }

    function selectDrone(id) {"""
content = content.replace("    function selectDrone(id) {", js_render_card)

js_select_drone = """    function selectDrone(id) {
      selId = id;
      document.querySelectorAll('.dc').forEach(c => c.classList.remove('sel'));
      const card = document.getElementById(`dc-${id}`);
      if (card) card.classList.add('sel');
      const d = drones[id];"""
content = content.replace("""    function selectDrone(id) {
      selId = id;
      const d = drones[id];""", js_select_drone)

js_upsert = """      // Telemetri log (son 5 kayıt)
      tlogHistory[id].push({
        t: Date.now(),
        alt: d.alt, spd: d.speed, roll: d.roll, pitch: d.pitch, yaw: d.yaw, bat: d.battery
      });
      if (tlogHistory[id].length > 5) tlogHistory[id].shift();

      renderCard(id);
      if (selId === id && activeTab === 'detail') renderDetailPanel(id);
      if (activeTab === 'all') renderAllPanel();
    }"""
content = content.replace("""      // Telemetri log (son 5 kayıt)
      tlogHistory[id].push({
        t: Date.now(),
        alt: d.alt, spd: d.speed, roll: d.roll, pitch: d.pitch, yaw: d.yaw, bat: d.battery
      });
      if (tlogHistory[id].length > 5) tlogHistory[id].shift();
    }""", js_upsert)

with open(file_path, "w") as f:
    f.write(content)
