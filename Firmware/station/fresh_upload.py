Import("env")

# Normal firmware updates preserve NVS identities, pairing and local archives.
# Full erasure is available only through the explicit factory-reset environment.
uploader_flags = list(env.get("UPLOADERFLAGS", []))
try:
    write_flash_index = uploader_flags.index("write-flash")
except ValueError:
    raise RuntimeError("esptool write-flash command was not configured")

erase_all = env.GetProjectOption("custom_erase_all", "false").lower() == "true"
uploader_flags = [flag for flag in uploader_flags if flag != "--erase-all"]
if erase_all:
    write_flash_index = uploader_flags.index("write-flash")
    uploader_flags.insert(write_flash_index + 1, "--erase-all")
elif env.GetProjectOption("custom_application_only", "false").lower() == "true":
    # This target updates an installed station without rewriting its partition
    # table, bootloader, OTA metadata or filesystem.
    env.Replace(FLASH_EXTRA_IMAGES=[], ESP32_APP_OFFSET="0x10000")
env.Replace(UPLOADERFLAGS=uploader_flags)
