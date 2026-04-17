import re

file_path = "/home/mell/Downloads/autonomous-multi-drone-swarm-px4/drone-command-center.html"
with open(file_path, "r") as f:
    content = f.read()

# 1. Remove .right HTML
html_pattern = r'    <div class="right">[\s\S]*?</div>\n      </div>\n    </div>'
# wait, .right is closed, then .main is closed.
# <div class="right"> ... </div>
# </div> <!-- map-wrap -->
# </div> <!-- main -->
content = re.sub(r'    <div class="right">.*?</div>\n  </div>\n  <footer>', r'  </div>\n  <footer>', content, flags=re.DOTALL)

# 2. Remove JS functions: switchTab, sparklineSVG, axisBarHTML, renderDetailPanel, renderAllPanel
js_funcs = r'    function switchTab\(tab\) \{.*?\n    \}[\s\S]*?function renderAllPanel\(\) \{.*?\n    \}'
content = re.sub(js_funcs, '', content, flags=re.DOTALL)

# 3. Remove from selectDrone:
content = re.sub(r'      // Her zaman render et — veri yoksa waiting mesajı göster\n      if \(activeTab === \'detail\'\) renderDetailPanel\(id\);\n      else if \(activeTab === \'all\'\) \{ switchTab\(\'detail\'\); \}', '', content)

# 4. Remove from upsertDrone:
content = re.sub(r'      if \(selId === id && activeTab === \'detail\'\) renderDetailPanel\(id\);\n      if \(activeTab === \'all\'\) renderAllPanel\(\);\n', '', content)

# 5. Remove setInterval
content = re.sub(r'    // Seçili drone panelini sürekli yenile \(100ms\)\n    setInterval\(\(\) => \{\n      if \(selId && activeTab === \'detail\' && drones\[selId\]\) renderDetailPanel\(selId\);\n      if \(activeTab === \'all\'\) renderAllPanel\(\);\n    \}, 100\);\n', '', content)

with open(file_path, "w") as f:
    f.write(content)
