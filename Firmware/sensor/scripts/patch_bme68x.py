"""Bound Bosch BME68x 4.4.8 mode switching on an acknowledged stuck sensor."""
from pathlib import Path
Import("env")
project = Path(env.subst("$PROJECT_DIR"))
root = Path(env.subst("$PROJECT_LIBDEPS_DIR"))
if not root.is_absolute():
    root = project / root
path = root / env.subst("$PIOENV") / "BME68x Sensor library/src/bme68x/bme68x.c"
source = path.read_text(encoding="utf-8")
if "WCHARGER_BME68X_MODE_DEADLINE" not in source:
    needle = "    /* Call until in sleep */\n    do\n    {"
    if source.count(needle) != 1:
        raise RuntimeError("BME68x mode-switch layout changed; review the pinned driver")
    source = source.replace(needle,
        "    /* WCHARGER_BME68X_MODE_DEADLINE: retain normal polling, cap faults. */\n"
        "    uint8_t mode_attempts = 0;\n"
        "    /* Call until in sleep */\n    do\n    {\n"
        "        if (++mode_attempts > 50) return BME68X_E_COM_FAIL;", 1)
    path.write_text(source, encoding="utf-8")
