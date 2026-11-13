# Validation record

Prepared on 2 November 2026 in a Linux x86_64 environment with GCC 13.3 and Python 3.12. These are software validation results, not measurements from a physical L1.

| Check | Result |
|---|---|
| Build/link SDK reader against supplied x86_64 archive | Passed |
| Build passive MAVLink reader | Passed |
| Standalone CMake configure/build | Passed |
| Automated unit and synthetic-serial tests | 15 passed |
| Python source/launch syntax; package XML and RViz YAML parsing | Passed |
| Physical L1 cloud and IMU acquisition | Not run: no sensor available |
| Electrical measurements | Not run: no sensor/supply/meter available |
| ROS 2 colcon build/runtime and RViz display | Not run: ROS 2 is not installed here |
| aarch64 build/runtime | Not run: x86_64 host only |

## What the automated tests cover

- Normal synthetic rates and values pass without setting `power_verified` true.
- A trailing dropout fails even when the average IMU rate is still high.
- Empty/nonfinite clouds, absent clouds and invalid IMU values fail.
- Warmup samples are excluded.
- 16-bit IMU ID wrapping is accepted; missing IDs and clock resets are recorded.
- A silent child process can be terminated within the supervisor deadline.
- Malformed worker JSON is counted.
- Valid MAVLink frames and a deliberately corrupted CRC are distinguished.
- The supplied **real SDK binary library** decodes cloud and IMU packets arriving over a Linux pseudoterminal.
- An end-to-end CLI run passes with sustained synthetic packets, preserving subsecond timestamp precision.
- A real SDK reader connected to a silent pseudoterminal times out and writes a failing report.
- Two readers from this package cannot own the same serial port simultaneously.

The synthetic fixture is deliberately simple: fixed two-metre distances and a unit quaternion. It tests the software path and failure detection; it does not validate real geometry, factory calibration, latency, IMU scale, electrical behavior, USB hardware or firmware compatibility.

## SDK buffering compatibility fix

Before the shim, the sustained synthetic test observed approximately 32 IMU messages/s from an approximately 200-message/s input, with ID skips and low cloud rate. Inspection of the bundled static archive showed `Serial::waitReadable()` being called before cached bytes were drained. Bypassing this one wait through GNU ld `--wrap` allows the existing SDK `available()` check and parser to progress. The same full-stream test then passes the default screening thresholds. This is a host-side workaround specific to the provided libraries, and still needs hardware validation.

## Repeat locally

```bash
./build_standalone.sh
# Or, without ROS sourced:
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

For the actual sensor, follow README sections 3–6. Save new hardware reports; no synthetic PASS report is supplied as evidence for your device.
