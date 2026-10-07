# Publication and privacy review — 2026-10-07

This review prepares firmware 4.3.4 and the changes after commit `1f690bc` /
[PR #30](https://github.com/Ans1S/esp32-c3-iot-sensor-board/pull/30) for public
GitHub publication. It is a targeted credential/privacy review, not an
application security audit or proof that every possible secret is absent.

## Current publication scope

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

## Corrections made

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

## Already published history

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
