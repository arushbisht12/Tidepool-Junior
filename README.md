# Tidepool-Junior: three-thread C exercise

Target: Zephyr on the `qemu_cortex_m3` emulated MCU. `src/main.c` contains
numbered TODOs; you write the implementation. It currently builds as an empty
application. No threads, channel operations, or fault logic are implemented yet.

## Build and run

From the parent workspace:

```sh
source .venv/bin/activate
west build -p always -b qemu_cortex_m3 Tidepool-Junior -d build/qemu
west build -d build/qemu -t run
```

After the first build, use `west build -d build/qemu` for incremental builds.
Exit QEMU with Ctrl+A, then X. `prj.conf` disables
instruction counting so delays follow wall-clock time more closely.
The Teensy overlay is not used for this target; no physical IMU is needed.

### CMake presets and clangd

Alternatively, from the parent workspace:

```sh
source .venv/bin/activate
cd Tidepool-Junior
cmake --preset qemu
cmake --build --preset qemu
```

The `qemu` preset uses `../build/qemu`; `board` uses `../build/junior`
and targets `teensy40`. Both use Ninja and export `compile_commands.json`.
In VS Code, use **CMake: Select Configure Preset** to choose the target.

clangd needs the build's compiler flags, include paths, and generated Zephyr
configuration, in addition to `#include <zephyr/kernel.h>`. `.clangd` points at
the live QEMU compilation database so it stays current after configuring.
After initial configuration, run **clangd: Restart language server** in VS Code.
Apple clangd 14 also needs an explicit `-mfloat-abi=soft` for this Cortex-M3.
Two newer clangd configuration options were removed because version 14 does
not recognize them.

To analyze Teensy instead, change `.clangd`'s `CompilationDatabase` to
`../build/junior` and remove its QEMU-specific `Add: [-mfloat-abi=soft]` line.
Configure/build the `board` preset and restart clangd. Selecting a CMake preset
does not automatically change the database selected in `.clangd`.

## Intended behavior

| Thread | Target period | Publishes | Reads |
| --- | --- | --- | --- |
| Estimation | 10 ms / 100 Hz | State | Simulated IMU |
| Control | 20 ms / 50 Hz | Command, control status | State, supervisor context |
| Main / supervisor | 1000 ms / 1 Hz | Context, readiness, latched fault | State, control status, optionally command |

Channels hold the latest complete message behind a mutex. Consumers may miss
intermediate publications; this is intentional. Start with everything in
`main.c`; split files once behavior and responsibilities are clear.

Supervisor starts INITIALIZING with control disabled. It enters READY after
fresh valid state and a fresh control heartbeat arrive. Inhibited control status
counts as a heartbeat, preventing a startup deadlock. Allow 3 seconds for
startup, then fault on missing data. After READY, invalid/stale state or a lost
controller heartbeat causes FAULT. Latch the first fault until reboot.
Controller independently inhibits on stale state/context before supervision
notices. Suggested age limits: state 50 ms, context 2500 ms, status 100 ms.
Use the original sample timestamp for state freshness and a received flag for
uninitialized channels.

## Implementation milestones

1. Define constants and types (TODOs 1–2). Build. Practice struct initialization
   and field access; understand `.` versus `->`.
2. Add periodic workers and supervisor heartbeats (TODOs 4, 7–8). Keep control
   disabled. Verify timing before adding data flow.
3. Add channels and startup synchronization (TODO 3). Practice passing a pointer
   so a function can copy a snapshot into caller-owned storage.
4. Implement fake IMU and state publication (TODO 5). State sequences should
   advance by roughly 100 per second.
5. Add controller snapshots, freshness checks and inhibited publications
   (TODO 6). Control cycles should advance by roughly 50 per second.
6. Add supervisor transitions and placeholder control. Print once per second.

Use absolute deadlines so work time does not accumulate into frequency drift.
Track overruns and skip missed releases. Measure over several seconds because
scheduling and console output add jitter.

## Acceptance experiments

- Normal boot: INITIALIZING then READY; enabled commands require fresh state
  and fresh READY context allowing control.
- Pause estimator publication after READY: control disables once state age
  exceeds 50 ms, within about one further control period. Supervisor eventually
  latches FAULT. Resuming estimation does not clear the fault.
- Publish invalid state: control inhibits even with a fresh timestamp.
- Pause context publication: control inhibits once context age exceeds 2500 ms,
  within about one further control period.
- Pause control-status publication: supervisor faults at its next check after
  status age exceeds 100 ms. A future actuator consumer would also need its own
  command-age timeout; this exercise has no actuator.
- Never publish state at boot: remain inhibited and fault when the startup
  grace period expires.
- Add work exceeding a period: observe overruns and skipped releases while
  other threads continue, without a busy catch-up loop.

For fault experiments, stop publication while continuing the timed loop.
Never pause or print while holding a mutex.
