# Firmware 4.4.4 release-build evidence

Base: main `1c6a4ba297b2feb743295eba12efc259605afea3` (merged PR #36).
Version 4.4.4 / 40404, ESP-NOW protocol V5. `build-artifacts.json` records final
application/ELF/partition and distributed package hashes. Unlike the earlier
three audit snapshots, these builds include the final release identity.

Both sensor payloads equal their final application builds byte for byte and
verify with the existing public header. The private/public identity agrees;
all nine previously retained sensor packages verify. Previous release bytes
remain unchanged. Slot sizes remain 0x140000 for sensors and 0x240000 for
station applications, beginning at 0x10000. The ZIP has exactly six members:
station application, two signed sensor packages, installation instructions,
release notes and its internal checksums. All release manifests are verified.

All three PlatformIO builds and all 30 sanitized host suites pass. Previous
normal host and complete browser audit results remain linked in the audit
reports; OTA/recording/interaction browser checks and package/upload/library/
syntax/publication regressions are checked for preparation. Publication scans
cover worktree, actual staged blobs and final commit before pushing.

Local raw release logs stay in ignored `.tools/release-4.4.4/`. Software evidence
does not prove physical timing, energy, RF, power-cut safety or stack watermarks.
No hardware was accessed, uploaded, erased or measured. GitHub PR publication
is separate from merging the PR or publishing a GitHub Release.
