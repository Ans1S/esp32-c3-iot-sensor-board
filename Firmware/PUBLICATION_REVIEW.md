# Publication and privacy review — 2026-10-25

## Firmware 4.4.1 publication

The publication baseline is `c9bf109`, the merge of
[PR #31](https://github.com/Ans1S/esp32-c3-iot-sensor-board/pull/31).
The new branch retains the existing 4.3.5, 4.4.0 and 4.4.1 work and packages,
then refreshes the root README and publication safeguards. A local ignored
snapshot preserves the pre-publication working copy.

### Review scope and result

- Scan every tracked or nonignored new file, including firmware bytes and ZIP
  members; repeat against the actual index and final commit before publishing.
  The working-tree pass covers 361 files and 72 archive members with no findings.
- Review credential assignments, provider tokens, private-key material,
  signed access URLs, personal build paths and private file names. A separate
  inventory checks email addresses, local-network addresses and MAC addresses.
  The remaining email is upstream license attribution; setup-network addresses
  and locally administered test MACs are intentional examples, not credentials.
- Inspect all public raster images and their metadata, including the five new
  embedded-page captures. Public images contain no EXIF/XMP metadata. The new
  views use fixed example measurements, generic node names, reserved example
  IP addresses and local demo MACs. Password/key fields remain empty. Original
  user screenshots are not copied into the repository.
- Inventory ignored local files. Signing keys, the installation public header,
  build/dependency caches, raw source renders, agent guidance, local snapshots
  and previous artifact archives remain outside Git. Add exclusions for device
  exports, NVS/flash dumps, raw captures, SSH/cloud authentication directories,
  client certificates, VPN files and package-registry credentials.
- Remove the broad factory-image exception. Permit only the reviewed versioned
  station applications despite the general binary exclusion. Explicit binary
  attributes and byte-preserving release text keep Git clones, ZIP members and
  SHA-256 manifests consistent across Windows/macOS/Linux. Required source,
  build scripts and key-initialization instructions stay public.
- Verify the existing installation key against the trusted header, verify all
  seven retained sensor packages and compare both current sensor payloads with
  final builds. Synchronize the station package with its final compiled image,
  then rebuild its complete ZIP and checksums. No installation key is replaced.
  All sixteen manifest entries and both update-bundle member sets match. A
  version-specific manifest covers the six current release download assets.

The seven publication regressions include ignored-data rejection, retained
public build inputs, staged/worktree disagreement, ZIP/binary scanning and
new-history findings. All 24 production host suites, eight embedded browser
suites, cryptographic/package/library/upload/script checks and all three
PlatformIO builds pass. No hardware was flashed or physically measured.

### Historical findings remain separate

The baseline reachable-history scan covers 652 unique file blobs and 72 archive
members. It reports 48 personal-path occurrences, 114 signed-download-URL
occurrences and one previously tracked local agent file. These are occurrences,
not counts of unique credentials. No provider-token or complete private-key
material is detected by these rules. URL validity and permissions are not
tested. The scan does not perform OCR or prove the absence of every secret.

The current exclusions do not remove already published Git objects. This
publication does not rewrite history or revoke third-party access. Retain the
earlier review below as the record of the original cleanup.

## Previous review — firmware 4.3.4, 2026-10-07

This review prepares firmware 4.3.4 and the changes after commit `1f690bc` /
[PR #30](https://github.com/Ans1S/esp32-c3-iot-sensor-board/pull/30) for public
GitHub publication. It is a targeted credential/privacy review, not an
application security audit or proof that every possible secret is absent.

### Previous publication scope

- Inspect all tracked/new publication candidates, actual staged Git blobs and
  the final commit tree. Checks include source/docs, hardware files, binary
  firmware and four ZIP archives containing 61 members.
- Check complete private-key blocks, common provider tokens, credential
  literals, signed access URLs, personal home/build paths and local-file names.
  Reports contain categories and locations only, never matched values.
- Inspect the three README dashboard/settings/OTA screenshots visually. Their
  device addresses, network, identifiers and IP are synthetic demo fixtures;
  credential fields are blank. Pixel-only text requires visual review because
  the byte scanner is not OCR.
- Verify the final V3/V4 OTA signatures with the existing installation public
  header, compare the signing identity with previous trusted packages, and
  match new payloads to their final build outputs. Match the station application
  to its final build and regenerate all three SHA-256 checksums.

No actual credentials, private signing-key material, signed access URLs or
personal build paths were detected in the cleaned publication candidates.
The ignored installation private key and public header remain local. Local
agent instructions, the project release skill, build outputs, raw screenshots,
device captures and the old release archive are outside the publication set.

### Corrections made

The initial review found personal compiler paths in the newly built firmware
and previous distribution binaries. Both projects now use
`shared/scripts/public_build_paths.py`; compiler prefix mapping covers project,
Arduino framework and dependency sources. The final applications and signed
packages pass the byte scan. See
[GCC's mapping behavior](https://gcc.gnu.org/onlinedocs/gcc/Preprocessor-Options.html#index-fmacro-prefix-map).

Eleven older path-bearing binaries were preserved in an ignored local archive
and withdrawn from the current release directory. Seven were previously
tracked; four were unpublished 4.3.2/4.3.3 intermediates. The current directory
contains only the two 4.3.4 OTA packages, the station application and their
documentation/checksums. No flash dump containing NVS or device recordings was
created or uploaded.

The previous Git hooks only scanned text and printed the matching source line.
They now run `tools/check_publication.py` against actual staged/pushed blobs and
newly introduced commit history, including binary bytes and ZIP members. A
working-tree edit cannot hide an already staged secret. Bare private-key PEM
markers in the TLS parser are distinguished from key material; the firmware
contains parser strings and the legitimate public verification key.

### Already published history

The baseline historical scan before creating this release covered 570 reachable
file blobs and the archive members.
It found old personal build paths and signed third-party download URLs, plus a
previously tracked agent-instruction file. No provider token or complete private
key material was detected by these rules. The signed URLs' current validity and
permissions were not tested; the review did not request them over the network.

Cleaning the current tree does **not** erase those already published objects.
This PR does not rewrite history, force-push old branches, revoke external
credentials or modify third-party accounts. Historical removal would require
a separately coordinated repository rewrite and cannot retract existing
clones or caches. Do not describe the entire historical repository as clean.

## Reproduce the checks

Run from the repository root:

```sh
python Firmware/tests/test_publication.py
python Firmware/tools/check_publication.py
python Firmware/tools/check_publication.py --staged
python Firmware/tools/check_publication.py --revision HEAD
python Firmware/tools/check_publication.py --history
git config core.hooksPath .githooks
```

The historical command intentionally reports the existing old findings. The
current worktree/index/commit checks must pass before publishing new changes.
The checker is heuristic; unrelated formats, encrypted archives, opaque secret
encodings and pixel-only text can require additional manual inspection.
