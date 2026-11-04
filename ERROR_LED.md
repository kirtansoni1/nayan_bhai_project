# Error codes

RED blinks N times, pauses, repeats. If more than one error is active, the highest number is shown.

| Blinks | Error | When it happens | How it clears | Effect on the machine |
|---|---|---|---|---|
| 1 | Sensor limit | `SENSOR_TRIGGER_COUNT` cycles in a row saw a detection | Next START that runs | Machine finishes the current cycle, then goes to STOPPED |
| 2 | Sensor blocked | START pressed while the sensor is active | Next START that runs | Cycle does not start |
| 3 | Unexpected restart | Brownout, crash or watchdog reset, tracked positions are lost | First START press (acknowledge only, does not start) | START and HOME are held until acknowledged |
| 4 | IO expander fault | Expander missing at boot, or a write still failing after 3 tries | Boot: once the expander answers. Running: stays on until a power cycle | Motion stops, DC motors set to 0, machine goes to STOPPED |

# Buttons

| Button | STOPPED | RUNNING | PAUSED | JOGGING |
|---|---|---|---|---|
| START | Checks sensor is clear, resets sensor count, starts a fresh cycle | Ignored | Resumes the cycle | Ignored |
| STOP | Ignored | Pauses the cycle | Ignored | Cancels the jog |
| HOME | Moves S_M3 `S_M3_JOG_STEPS` in `S_M3_JOG_DIR`, end point becomes S_M3 zero | Refused | Refused | Ignored |

A cancelled jog also sets S_M3 zero wherever it stopped.

# Sensor counting

- Detections count only from the end of task 9 until the end of the cycle (task 10 and `CYCLE_GAP_MS`).
- Only one detection counts per cycle. An object already in front of the sensor when the window opens counts.
- Detections while paused inside the window count.
- A cycle with no detection resets the count to 0.
- When the count reaches `SENSOR_TRIGGER_COUNT`, the cycle runs to its end and the machine stops.

Example with `SENSOR_TRIGGER_COUNT = 2`:

| Cycle | Detected in window | Count after | Result |
|---|---|---|---|
| 1 | no | 0 | continue |
| 2 | yes | 1 | continue |
| 3 | no | 0 | continue |
| 4 | yes | 1 | continue |
| 5 | yes | 2 | cycle 5 finishes, machine stops |
