Import("env")

# Normal firmware updates preserve NVS identities, pairing and local archives.
# Full erasure is available only through the explicit factory-reset environment.
uploader_flags = list(env.get("UPLOADERFLAGS", []))
try:
    write_flash_index = uploader_flags.index("write-flash")
except ValueError:
    raise RuntimeError("esptool write-flash command was not configured")

erase_all = env.GetProjectOption("custom_erase_all", "false").lower() == "true"
if erase_all and "--erase-all" not in uploader_flags:
    uploader_flags.insert(write_flash_index + 1, "--erase-all")
env.Replace(UPLOADERFLAGS=uploader_flags)
