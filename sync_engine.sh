#!/usr/bin/env bash
# Builds the controller against the engine's Hazel DLLs and, only if that succeeds, mirrors src/*.cs into the engine project
# (Assets/Scripts/Client/Source/G1BoxGrasp/) with the engine's ArmReach (0.65; the repo keeps 0.50). Then just open or Play the editor.
set -euo pipefail
cd "$(dirname "$0")"
client=${ENGINE_CLIENT:-$HOME/Documents/LuckyEngine/Projects/g1_sort/Assets/Scripts/Client}
dest=$client/Source/G1BoxGrasp

dotnet build G1BoxGrasp.csproj -nologo -v q

# Mirror the sources (only .cs: the engine compiles every .cs under Source/)
rm -rf "$dest"
(cd src && find . -name '*.cs' -exec install -D -m 644 {} "$dest/{}" \;)
sed -i 's/public float ArmReach = 0.50f;/public float ArmReach = 0.65f;/' "$dest/Config/Settings.cs"
grep -q 'public float ArmReach = 0.65f;' "$dest/Config/Settings.cs" || { echo "ArmReach not set in the engine copy" >&2; exit 1; }
rm -f "$client/Source/g1_grasp.cs" "$client/Source/g1_grasp.episodes.cs" # the old single-file version

# Keep the engine's file list current (the editor regenerates it the same way from Source/**.cs when it starts)
python3 - "$client/G1Sort.csproj" "$dest" <<'PY'
import re, sys
from pathlib import Path
proj, dest = Path(sys.argv[1]), Path(sys.argv[2])
text = proj.read_text()
text = re.sub(r'\s*<Compile Include="Source\\(g1_grasp[^"]*|G1BoxGrasp\\[^"]*)" />', "", text)
new = "".join(f'\n    <Compile Include="Source\\G1BoxGrasp\\{p.relative_to(dest).as_posix().replace("/", chr(92))}" />' for p in sorted(dest.rglob("*.cs")))
i = text.rindex("\n  </ItemGroup>")
proj.write_text(text[:i] + new + text[i:])
PY
echo "synced $(find "$dest" -name '*.cs' | wc -l) files to $dest"
