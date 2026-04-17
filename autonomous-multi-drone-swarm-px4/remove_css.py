import re

file_path = "/home/mell/Downloads/autonomous-multi-drone-swarm-px4/drone-command-center.html"
with open(file_path, "r") as f:
    content = f.read()

pattern = r'    \.right \{[\s\S]*?    footer \{'
content = re.sub(pattern, '    footer {', content)

with open(file_path, "w") as f:
    f.write(content)
