# Selected actual machine-code changes

These excerpts are copied from the linked before/after ELF disassemblies, using
the tool versions in `toolchain.json`. Addresses identify these exact builds;
the ELF hashes are in `artifact-manifest.json`. Omitted instructions are marked.
The changes are in C++; the compiler output verifies their native effect.
These are static excerpts, not cycle measurements or execution traces.

## RISC-V C3 V4: OTA local frame

Before, `sensor::checkOta()` allocates its frame in two adjustments:

```asm
4200d43e: 81010113  addi sp,sp,-2032
# ... saved registers and constant setup ...
4200d478: 716d      addi sp,sp,-272
```

After, the existing heap sector buffer replaces the prefix stack buffer:

```asm
4200d412: b0010113  addi sp,sp,-1280
```

The local frame is 2032 + 272 = 2304 bytes before, versus 1280 after. Both
values agree with the separate GCC `-fstack-usage` probes. Caller depth and
interrupt/task stack usage are outside this local-frame comparison.

## RISC-V C3 V4: radio packet copies and CRC calls

Before, `tryChannel()` has two full-copy sites and two packet-finalization
sites: the first pair precedes the attempts; the second runs per actual send.

```asm
4200991c: 0cf00613  li    a2,207
42009920: 85a2      mv    a1,s0
42009922: 1008      addi  a0,sp,32
42009924: fe384097  auipc ra,0xfe384
42009928: 97c080e7  jalr  -1668(ra) # 4038d2a0 <memcpy>
# ...
420099ac: 916f90ef  jal   42002ac2 <lil::protocol::finalizePacket(...)>
# ... retry/response logic ...
42009a32: 0cf00613  li    a2,207
42009a36: 85a2      mv    a1,s0
42009a38: 1008      addi  a0,sp,32
42009a3a: fe384097  auipc ra,0xfe384
42009a3e: 866080e7  jalr  -1946(ra) # 4038d2a0 <memcpy>
# ... relative-age updates ...
42009ad0: ff3f80ef  jal   42002ac2 <lil::protocol::finalizePacket(...)>
```

After, one copy/finalization pair remains per actual send. The third `memcpy`
argument is the selected wire length, loaded from the stack, rather than the
constant full size. Source dataflow selects 60/95/207 bytes; bytewise host
comparisons verify all three formats across retries, including full-packet ages.

```asm
42009a40: 4652      lw    a2,20(sp)
42009a42: 03c10d93  addi  s11,sp,60
42009a46: 85a2      mv    a1,s0
42009a48: 856e      mv    a0,s11
42009a4a: fe384097  auipc ra,0xfe384
42009a4e: 856080e7  jalr  -1962(ra) # 4038d2a0 <memcpy>
# ... included relative-age updates ...
42009aca: ff9f80ef  jal   42002ac2 <lil::protocol::finalizePacket(...)>
42009ace: 0310000f  fence rw,w
```

The complete function shrinks from 952 to 890 bytes and 320 to 292 static
instructions. Its local frame increases from 304 to 336 bytes. Per-attempt
copies for environmental packets shrink 207 -> 60 bytes; physical wire length
remains 60 bytes. Release/acquire send-completion ordering remains present.

## Xtensa S3: station registry frame

The direct telemetry assignment removes the intermediate packed payload copy.
The actual `SensorRegistry::registerTelemetry()` entries are:

```asm
# Before
4201e078: 03a136  entry a1, 0x1d0
# After
4201e1e4: 024136  entry a1, 0x120
```

Hexadecimal 0x1d0 = 464 and 0x120 = 288 bytes: a 176-byte local-frame saving.
The linked body shrinks 1532 -> 1496 bytes and 572 -> 565 static instructions.
Both frames agree with GCC. Host tests retain input bytes, duplicate/generation
behavior and BME680 raw-fallback IAQ/accuracy preservation.

All selected bodies, both C3 variants and unchanged station main setup/loop
costs appear in `function-comparison.csv`. Full local disassemblies remain under
`.tools/firmware-flow-20261107/{before,after}/{target}/disassembly.txt`.
