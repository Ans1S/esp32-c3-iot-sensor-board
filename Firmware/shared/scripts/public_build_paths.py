"""Remove personal build paths from compiled diagnostics and debug metadata."""
from pathlib import Path

Import("env")

# Include framework/dependency sources below the build user's home, not just
# this project: their __FILE__ diagnostics also become firmware strings.
project = Path(env.subst("$PROJECT_DIR")).resolve()
prefixes = [(Path.home(), "build-user"), (project.parent.parent, "wcharger")]
flags = []
for source, target in prefixes:
    for spelling in sorted({str(source), source.as_posix()}):
        flags.append(f"-ffile-prefix-map={spelling}={target}")
env.Append(CCFLAGS=flags)
