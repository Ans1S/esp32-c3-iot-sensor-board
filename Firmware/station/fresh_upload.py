Import("env")

# PlatformIO places normal upload_flags before the esptool subcommand. The
# --erase-all option belongs specifically to `write-flash`, so insert it at
# the valid position without introducing a separate, failure-prone erase job.
uploader_flags = list(env.get("UPLOADERFLAGS", []))
try:
    write_flash_index = uploader_flags.index("write-flash")
except ValueError:
    raise RuntimeError("esptool write-flash command was not configured")

if "--erase-all" not in uploader_flags:
    uploader_flags.insert(write_flash_index + 1, "--erase-all")
env.Replace(UPLOADERFLAGS=uploader_flags)
