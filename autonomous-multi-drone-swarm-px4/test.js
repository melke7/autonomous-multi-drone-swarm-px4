
    const HOME = { lat: 37.4135, lon: -121.9966 };
    const GRID_COLS = 20, GRID_ROWS = 15;
    let drones = {}, markers = {}, trails = {}, gridLayers = [];
    let simRunning = false, simInterval = null, simTime = 0;
    let ws = null, selId = null, activeTab = 'detail';
    let tlogHistory = {};

    const map = L.map('map', { center: [HOME.lat, HOME.lon], zoom: 18, zoomControl: false, maxZoom: 19 });

    // Tile katmanları — Esri Hybrid (uydu + etiket) varsayılan
    const esriSat = L.tileLayer(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      { attribution: '© Esri © Maxar © Airbus', maxZoom: 19 }
    );
    const esriLabels = L.tileLayer(
      'https://services.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}',
      { maxZoom: 19, pane: 'overlayPane' }
    );
    const cartoDark = L.tileLayer(
      'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
      { attribution: '© CARTO © OSM', subdomains: 'abcd', maxZoom: 19 }
    );
    const esriPureSat = L.tileLayer(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      { attribution: '© Esri', maxZoom: 19 }
    );

    const TILE_MODES = [
      { key: 'hybrid', label: 'HYBRID', layers: [esriSat, esriLabels] },
      { key: 'dark', label: 'DARK', layers: [cartoDark] },
      { key: 'sat', label: 'SAT', layers: [esriPureSat] },
    ];
    let tileIdx = 0;

    function applyTileMode(idx) {
      // Mevcut katmanları kaldır
      [esriSat, esriLabels, cartoDark, esriPureSat].forEach(l => { try { map.removeLayer(l); } catch { } });
      TILE_MODES[idx].layers.forEach(l => l.addTo(map));
      const lbl = TILE_MODES[idx].label;
      document.getElementById('tileBtn').textContent = '🗺 ' + lbl;
    }

    function cycleTile() {
      tileIdx = (tileIdx + 1) % TILE_MODES.length;
      applyTileMode(tileIdx);
    }

    // Başlangıç: Hybrid
    applyTileMode(0);

    L.control.zoom({ position: 'bottomleft' }).addTo(map);
    map.on('moveend zoomend', updateTrafficGrid);

    function droneIcon(hdg, status) {
      const c = status === 'crit' ? '#f87171' : status === 'warn' ? '#fbbf24' : '#34d399';
      const glow = status === 'crit' ? 'rgba(248,113,113,0.5)' : status === 'warn' ? 'rgba(251,191,36,0.4)' : 'rgba(45,212,191,0.4)';
      const svg = `<svg width="32" height="32" viewBox="0 0 32 32" xmlns="http://www.w3.org/2000/svg"
    style="transform:rotate(${hdg}deg);filter:drop-shadow(0 0 5px ${glow})">
    <circle cx="16" cy="16" r="14" fill="rgba(8,13,20,0.75)" stroke="${c}" stroke-width="1.5"/>
    <polygon points="16,4 21,22 16,19 11,22" fill="${c}"/>
    <circle cx="16" cy="16" r="2.5" fill="${c}"/>
    <line x1="7" y1="7" x2="11" y2="11" stroke="${c}" stroke-width="1.5"/>
    <line x1="25" y1="7" x2="21" y2="11" stroke="${c}" stroke-width="1.5"/>
    <line x1="7" y1="25" x2="11" y2="21" stroke="${c}" stroke-width="1.5"/>
    <line x1="25" y1="25" x2="21" y2="21" stroke="${c}" stroke-width="1.5"/>
    <circle cx="11" cy="11" r="2" fill="none" stroke="${c}" stroke-width="1"/>
    <circle cx="21" cy="11" r="2" fill="none" stroke="${c}" stroke-width="1"/>
    <circle cx="11" cy="21" r="2" fill="none" stroke="${c}" stroke-width="1"/>
    <circle cx="21" cy="21" r="2" fill="none" stroke="${c}" stroke-width="1"/>
  </svg>`;
      return L.divIcon({ html: svg, className: '', iconSize: [24, 24], iconAnchor: [12, 12] });
    }

    function upsertDrone(id, data) {
      if (!drones[id]) {
        drones[id] = {
          id, lat: HOME.lat, lon: HOME.lon, alt: 0, speed: 0, battery: 100, heading: 0,
          roll: 0, pitch: 0, yaw: 0, throttle: 0, status: 'ok', trail: [], ts: Date.now()
        };
        tlogHistory[id] = [];
      }
      Object.assign(drones[id], data);
      drones[id].ts = Date.now();
      const d = drones[id];
      d.status = d.battery < 20 ? 'crit' : d.battery < 40 ? 'warn' : 'ok';

      // Aynı konumdaki drone'ları küçük offset ile ayır (SITL spawn jitter)
      // Her drone ID'sine özgü sabit mikro-offset (lat ~1m, lon ~1m per step)
      const jLat = (id - 1) * 0.000009;   // ~1 m kuzey
      const jLon = ((id - 1) % 3) * 0.000012 - 0.000012; // ~1m doğu/batı
      const ll = [d.lat + jLat, d.lon + jLon];
      if (!markers[id]) {
        markers[id] = L.marker(ll, { icon: droneIcon(d.heading, d.status), zIndexOffset: 500 + id })
          .bindTooltip('', { direction: 'top', className: 'dtip', offset: [0, -12] })
          .addTo(map)
          .on('click', () => selectDrone(id));
        trails[id] = L.polyline([], { color: '#2dd4bf', weight: 1.2, opacity: 0.3, dashArray: '4,7' }).addTo(map);
      }
      markers[id].setLatLng(ll);
      markers[id].setIcon(droneIcon(d.heading, d.status));
      markers[id].setTooltipContent(
        `<b>UAV-${String(id).padStart(2, '0')}</b><br>` +
        `ALT: ${d.alt.toFixed(1)}m &nbsp; SPD: ${d.speed.toFixed(1)}m/s<br>` +
        `BAT: ${d.battery.toFixed(0)}% &nbsp; HDG: ${d.heading.toFixed(0)}°<br>` +
        `ROLL: ${d.roll.toFixed(1)}° &nbsp; PITCH: ${d.pitch.toFixed(1)}°`
      );
      d.trail.push([d.lat, d.lon]);
      if (d.trail.length > 120) d.trail.shift();
      trails[id].setLatLngs(d.trail);

      // Telemetri log (son 5 kayıt)
      tlogHistory[id].push({
        t: Date.now(),
        alt: d.alt, spd: d.speed, roll: d.roll, pitch: d.pitch, yaw: d.yaw, bat: d.battery
      });
      if (tlogHistory[id].length > 5) tlogHistory[id].shift();

      renderCard(id);
    }

    function updateTrafficGrid() {
      gridLayers.forEach(l => map.removeLayer(l));
      gridLayers = [];
      const b = map.getBounds();
      const latS = (b.getNorth() - b.getSouth()) / GRID_ROWS;
      const lonS = (b.getEast() - b.getWest()) / GRID_COLS;
      const grid = {};
      Object.values(drones).forEach(d => {
        const col = Math.floor((d.lon - b.getWest()) / lonS);
        const row = Math.floor((d.lat - b.getSouth()) / latS);
        if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
          const k = `${row},${col}`; grid[k] = (grid[k] || 0) + 1;
        }
      });
      let lo = 0, mid = 0, hi = 0;
      Object.entries(grid).forEach(([k, cnt]) => {
        const [row, col] = k.split(',').map(Number);
        const sw = [b.getSouth() + row * latS, b.getWest() + col * lonS];
        const ne = [sw[0] + latS, sw[1] + lonS];
        let color, fo, so;
        if (cnt >= 4) { color = '#f87171'; fo = .45; so = .35; hi++; }
        else if (cnt >= 2) { color = '#fbbf24'; fo = .32; so = .28; mid++; }
        else { color = '#34d399'; fo = .18; so = .2; lo++; }
        const r = L.rectangle([sw, ne], { color, weight: .7, fillColor: color, fillOpacity: fo, opacity: so }).addTo(map);
        gridLayers.push(r);
      });
      const total = lo + mid + hi;
      document.getElementById('sZones').textContent = total ? `${lo}L ${mid}M ${hi}H` : '—';
    }



    function renderCard(id) {
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

    function selectDrone(id) {
      selId = id;
      document.querySelectorAll('.dc').forEach(c => c.classList.remove('sel'));
      const card = document.getElementById(`dc-${id}`);
      if (card) card.classList.add('sel');
      const d = drones[id];
      if (d) map.panTo([d.lat, d.lon], { animate: true });

    }



    function updateStats() {
      const list = Object.values(drones);
      document.getElementById('sA').textContent = list.length;
      if (!list.length) return;
      const avg = fn => (list.reduce((s, d) => s + fn(d), 0) / list.length).toFixed(1);
      document.getElementById('sAlt').textContent = avg(d => d.alt) + ' m';
      document.getElementById('sSpd').textContent = avg(d => d.speed) + ' m/s';
      document.getElementById('sBat').textContent = avg(d => d.battery) + ' %';
    }

    setInterval(() => {
      document.getElementById('clock').textContent = new Date().toISOString().slice(11, 19);
    }, 500);

    const SIM_N = 7;
    const PATS = [
      (t, i) => ({ lat: HOME.lat + .0022 * Math.sin(t * .55 + i), lon: HOME.lon + .0032 * Math.cos(t * .55 + i) }),
      (t, i) => ({ lat: HOME.lat + .0028 * Math.sin(t * .35 + i * .8), lon: HOME.lon + .0028 * Math.sin(t * .7 + i * .8) }),
      (t, i) => { const r = .001 + .0018 * (((t * .08 + i) % (2 * Math.PI)) / (2 * Math.PI)); return { lat: HOME.lat + r * Math.sin(t * .48 + i), lon: HOME.lon + r * Math.cos(t * .48 + i) } },
      (t, i) => { const a = .003, ang = t * .42 + i, den = 1 + Math.sin(ang) ** 2; return { lat: HOME.lat + a * Math.cos(ang) / den, lon: HOME.lon + a * Math.cos(ang) * Math.sin(ang) / den } },
      (t, i) => ({ lat: HOME.lat + .002 * Math.sin(t * .28 + i * 2.1) * Math.cos(t * .62), lon: HOME.lon + .0025 * Math.cos(t * .32 + i * 1.6) * Math.sin(t * .5) }),
      (t, i) => ({ lat: HOME.lat + .0015 * Math.sin(t * .9 + i) + .001 * Math.cos(t * 1.3 + i), lon: HOME.lon + .0025 * Math.cos(t * .7 + i) + .0008 * Math.sin(t * 1.1 + i) }),
      (t, i) => ({ lat: HOME.lat + .0032 * Math.sin(t * .22 + i + 1.2) * Math.sin(t * .44 + i), lon: HOME.lon + .002 * Math.cos(t * .33 + i + .8) }),
    ];

    function startSim() {
      simRunning = true;
      document.getElementById('simBtn').classList.add('on');
      document.getElementById('simBtn').textContent = '⚙ SIM ACTIVE';
      document.getElementById('dot').classList.add('live');
      document.getElementById('statusLbl').textContent = 'SIM ACTIVE';
      document.getElementById('modeLbl').textContent = 'SIMULATION';
      for (let i = 1; i <= SIM_N; i++) {
        const p = PATS[(i - 1) % PATS.length](0, i);
        upsertDrone(i, {
          lat: p.lat, lon: p.lon, alt: 30 + i * 9, speed: 2 + i * 1.2, battery: 85 + Math.random() * 15,
          heading: 0, roll: 0, pitch: 0, yaw: i * 50, throttle: 40 + i * 5
        });
      }
      simInterval = setInterval(() => {
        simTime += .1;
        for (let i = 1; i <= SIM_N; i++) {
          const d = drones[i]; if (!d) continue;
          const p = PATS[(i - 1) % PATS.length](simTime, i);
          const dx = p.lon - d.lon, dy = p.lat - d.lat;
          const hdg = Math.atan2(dx, dy) * 180 / Math.PI;
          const spd = Math.sqrt(dx * dx + dy * dy) * 111000 / .1;
          const roll = 18 * Math.sin(simTime * .8 + i);
          const pitch = 12 * Math.cos(simTime * .6 + i);
          const yaw = (d.yaw + 2) % 360;
          const thr = Math.min(100, Math.max(0, 50 + 25 * Math.sin(simTime * .3 + i)));
          upsertDrone(i, {
            lat: p.lat, lon: p.lon,
            alt: 25 + i * 8 + 14 * Math.sin(simTime * .25 + i),
            speed: Math.min(22, Math.abs(spd)),
            battery: Math.max(0, d.battery - .0025),
            heading: hdg, roll, pitch, yaw, throttle: thr
          });
        }
        updateTrafficGrid(); updateStats();
      }, 100);
      if (!selId) { selId = 1; selectDrone(1); }
    }

    function stopSim() {
      clearInterval(simInterval); simRunning = false;
      document.getElementById('simBtn').classList.remove('on');
      document.getElementById('simBtn').textContent = '⚙ SIM MODE';
      document.getElementById('dot').classList.remove('live');
      document.getElementById('statusLbl').textContent = 'DISCONNECTED';
      document.getElementById('modeLbl').textContent = 'STANDBY';
    }
    function toggleSim() { if (simRunning) stopSim(); else { if (ws) ws.close(); startSim(); } }

    function connectWS() {
      if (simRunning) stopSim();
      const url = document.getElementById('wsUrl').value.trim();
      if (!url) return;
      if (ws) { ws.close(); ws = null; }
      try {
        ws = new WebSocket(url);
        ws.onopen = () => {
          document.getElementById('dot').classList.add('live');
          document.getElementById('statusLbl').textContent = 'LIVE';
          document.getElementById('modeLbl').textContent = 'LIVE DATA';
        };
        ws.onclose = () => {
          document.getElementById('dot').classList.remove('live');
          document.getElementById('statusLbl').textContent = 'DISCONNECTED';
          document.getElementById('modeLbl').textContent = 'STANDBY';
        };
        ws.onerror = () => { document.getElementById('statusLbl').textContent = 'ERROR'; };
        ws.onmessage = e => { try { parseMav(JSON.parse(e.data)); } catch { } };
      } catch (e) { document.getElementById('statusLbl').textContent = 'CONN FAILED'; }
    }

    function parseMav(msg) {
      const sid = msg?.header?.system_id;
      if (!sid || sid === 255) return;
      const type = msg?.message?.type, m = msg?.message;
      if (type === 'GLOBAL_POSITION_INT') {
        const vx = m.vx / 100, vy = m.vy / 100;
        upsertDrone(sid, {
          lat: m.lat / 1e7, lon: m.lon / 1e7, alt: m.relative_alt / 1000,
          speed: Math.sqrt(vx * vx + vy * vy), heading: m.hdg ? m.hdg / 100 : 0
        });
      } else if (type === 'SYS_STATUS') {
        const br = m.battery_remaining;
        if (br >= 0) upsertDrone(sid, { battery: br });
        else if (m.voltage_battery > 0)
          upsertDrone(sid, { battery: Math.max(0, Math.min(100, ((m.voltage_battery / 1000 - 3.3) / .9) * 100)) });
      } else if (type === 'VFR_HUD') {
        upsertDrone(sid, { speed: m.groundspeed, alt: m.alt, throttle: m.throttle * 100 });
      } else if (type === 'ATTITUDE') {
        const deg = r => r * 180 / Math.PI;
        upsertDrone(sid, { roll: deg(m.roll), pitch: deg(m.pitch), yaw: (deg(m.yaw) + 360) % 360, heading: (deg(m.yaw) + 360) % 360 });
      }
      updateTrafficGrid(); updateStats();
    }


    // Sayfa açılınca WebSocket bağlantısını otomatik dene,
    // bridge çalışıyorsa LIVE'a geçer; yoksa SIM başlar.
    function autoConnect() {
      const url = document.getElementById('wsUrl').value.trim();
      const ws_test = new WebSocket(url);
      ws_test.onopen = () => {
        ws_test.close();
        connectWS();
        document.getElementById('modeLbl').textContent = 'LIVE DATA';
      };
      ws_test.onerror = () => {
        startSim();
      };
    }
    setTimeout(autoConnect, 600);
