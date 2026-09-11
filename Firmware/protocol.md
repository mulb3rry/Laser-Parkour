# Laser Parkour sensor-bus protocol

This document defines version 1.1 of the I2C application protocol between the
Raspberry Pi Pico W controller and all ATtiny85 nodes. It is the normative
reference for both firmware targets. Protocol constants and data layouts must
be implemented once in a shared C header.

Version 1 uses a compact register map for ordinary data and configuration. A
small command mailbox is used only for operations with side effects. This
keeps frequent polling short while retaining explicit acknowledgements and
safe retry behavior where it is needed.

## 1. Physical bus

- The Pico W is the only I2C controller (master). Nodes are targets (slaves)
  and never initiate I2C traffic.
- The sensor bus runs at 10 kHz. It was reduced from the initial 100 kHz after
  communication failures appeared on the assembled branched wiring.
- The controller accepts at most 16 laser nodes, one start node, and one finish
  node. Normal operation is expected to use about 8–12 total nodes on a
  branched harness representative of the actual installation.
- Node PB0 is SDA and PB2 is SCL. Pico GPIO 16 is SDA and GPIO 17 is SCL
  (I2C0).
- `FU` is a separate shared active-low, open-drain event line on node PB4 and
  Pico GPIO 19. Only start and finish nodes may assert it.
- Pico GPIO18 does not provide a usable node reset because the reset jumpers on
  the assembled node PCBs are not fitted. Firmware must not rely on `AT_RS`.
- Multi-byte integers use little-endian byte order.

The final bus speed must be validated with the representative normal inventory.
Inventories up to 18 nodes remain valid but have a slower polling cycle. Tested
node count, branch lengths, rise times, transaction errors, polling duration,
and retry counts must be recorded.

## 2. Addressing and discovery

| Address range | Purpose |
|---|---|
| `0x08` | Uncommissioned node |
| `0x09`–`0x0F` | Reserved for commissioning or a future bootloader |
| `0x10`–`0x6F` | Commissioned nodes |
| All others | Not used by this protocol |

Only one uncommissioned node may be connected during commissioning. A normal
setup scan probes `0x08` and every configured or expected address. A full
`0x10`–`0x6F` scan is available for discovery and recovery.

The controller rejects duplicate addresses, incompatible protocol versions,
more than 16 laser nodes, more than one start node, or more than one finish
node. Exactly one start node, one finish node, and at least one laser node are
required before a game can be armed.

## 3. Register transport

Every transaction starts by writing a one-byte register address. Reads use the
standard combined-register transaction:

```text
START  node+W  register  REPEATED_START  node+R  register-data  STOP
```

The controller always writes the register pointer immediately before a read;
it does not rely on a pointer retained from an earlier transaction.

Register data has a fixed size. Multi-byte registers are captured as a coherent
snapshot when their register pointer is selected. Changes occurring during the
read become visible on the next transaction, never halfway through the current
one.

The node transmits the documented number of bytes. The last byte of every
defined register block is its CRC. A short read, extra read, incorrect CRC, or
unexpected value is a failed transaction. Reserved or unknown register
addresses return `0xFF` and have no defined block length.

Writable blocks are sent in one transaction:

```text
START  node+W  register  complete-data  crc  STOP
```

The node changes staged state only after receiving the complete block and
validating its CRC and every value. Partial, oversized, or invalid writes leave
the previous staged and active values unchanged. Ordinary register writes have
no sequence number; the controller verifies them by reading the staged block
back.

## 4. CRC

Register blocks use CRC-8/ATM (`CRC-8/SMBUS`):

- polynomial: `0x07` (`x^8 + x^2 + x + 1`);
- initial value: `0x00`;
- no input or output reflection;
- final XOR: `0x00`.

For both reads and writes, the CRC covers the one-byte register address followed
by every data byte. It does not include the I2C address or read/write bit. By
including the requested register, the controller can detect a corrupted
register-pointer write even if the selected block happens to have the same
length.

Example: register address `0x10` followed by data bytes `01 00 02 00 03 00`
has CRC `0xE9`:

```text
10 01 00 02 00 03 00 E9
```

## 5. Common values

### 5.1 Node role

| Value | Name | Meaning |
|---:|---|---|
| 0 | `UNCONFIGURED` | Identity has not been commissioned |
| 1 | `LASER` | LDR beam sensor |
| 2 | `START` | Start button |
| 3 | `FINISH` | Finish button |

All other role values are invalid in protocol version 1.

### 5.2 Operating mode

| Value | Name | Meaning |
|---:|---|---|
| 0 | `SETUP` | Configuration, diagnostics, and visible node status |
| 1 | `GAME` | Event detection active; normal node LED indications remain active |

Nodes always boot into `SETUP`. Operating mode is not persisted.

### 5.3 Input state

| Value | Name | Meaning |
|---:|---|---|
| 0 | `INACTIVE` | Laser clear or button released |
| 1 | `ACTIVE` | Laser broken or button pressed |
| 2 | `UNSTABLE` | Input has not met the stable-time requirement |
| 255 | `NOT_APPLICABLE` | Role/configuration has no valid input |

### 5.4 Result codes

| Value | Name | Meaning |
|---:|---|---|
| 0 | `OK` | Operation completed |
| 1 | `UNKNOWN_REGISTER` | Register address is unsupported |
| 2 | `INVALID_LENGTH` | Written block length is incorrect |
| 3 | `INVALID_CRC` | Written block CRC is incorrect |
| 4 | `INVALID_VALUE` | A field is outside its allowed range |
| 5 | `WRONG_ROLE` | Operation is not valid for this role |
| 6 | `WRONG_MODE` | Operation is not allowed in the current mode |
| 7 | `NOT_COMMISSIONING` | Identity change requires address `0x08` |
| 8 | `NO_STAGED_CONFIG` | Save requested without a valid staged change |
| 9 | `EEPROM_FAILURE` | Persisted data failed verification |
| 10 | `SEQUENCE_CONFLICT` | Sequence reused with different command content |
| 11 | `PROTECTION_FAILED` | Protected-command key is invalid |
| 12 | `BUSY` | Node cannot complete the operation yet |
| 13 | `UNKNOWN_COMMAND` | Mailbox command ID is unsupported |
| 14–255 | Reserved | Must not be emitted by version 1 firmware |

### 5.5 Status flags

| Bit | Mask | Name | Meaning when set |
|---:|---:|---|---|
| 0 | `0x0001` | `CONFIG_VALID` | EEPROM configuration is valid |
| 1 | `0x0002` | `COMMISSIONED` | Role and address are commissioned |
| 2 | `0x0004` | `GAME_MODE` | Node is in `GAME` mode |
| 3 | `0x0008` | `INPUT_ACTIVE` | Accepted input state is active |
| 4 | `0x0010` | `INPUT_UNSTABLE` | A stable-time transition is pending |
| 5 | `0x0020` | `COOLDOWN_ACTIVE` | Laser interruption cooldown is active |
| 6 | `0x0040` | `CONFIG_STAGED` | Valid unsaved sensor configuration exists |
| 7 | `0x0080` | `IDENTITY_STAGED` | Valid unsaved identity exists |
| 8 | `0x0100` | `LAST_OPERATION_ERROR` | Most recent write/command failed |
| 9 | `0x0200` | `ADC_RANGE_WARNING` | Laser ADC remains near an endpoint |
| 10 | `0x0400` | `COUNTER_OVERFLOWED` | Event counter wrapped after last reset |
| 11–15 | — | Reserved | Transmit as zero; ignore when receiving |

`LAST_OPERATION_ERROR` is cleared by the next successful write or mailbox
command. Reads do not change it.

## 6. Register map

Lengths below include the final CRC byte but exclude the register pointer sent
by the controller.

| Address | Name | Access | Length | Purpose |
|---:|---|---|---:|---|
| `0x00` | `IDENTITY` | Read | 11 | Protocol, firmware, role, and capabilities |
| `0x10` | `FAST_STATUS` | Read | 7 | Frequent event polling and restart detection |
| `0x18` | `DIAGNOSTICS` | Read | 10 | ADC, input, mode, error, and cooldown details |
| `0x20` | `ACTIVE_SENSOR_CONFIG` | Read | 9 | Active laser configuration |
| `0x30` | `STAGED_SENSOR_CONFIG` | Read/write | 9 | Configuration awaiting save |
| `0x40` | `STAGED_IDENTITY` | Read/write | 3 | Address and role awaiting save |
| `0xF0` | `COMMAND` | Write | 7 after pointer | Side-effecting operation mailbox |
| `0xF1` | `COMMAND_RESULT` | Read | 6 | Result of the latest mailbox command |

Undefined addresses are reserved. Existing register addresses and layouts do
not change within protocol major version 1.

### 6.1 `0x00 IDENTITY`

Read-only data before CRC:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Protocol major (`1`) |
| 1 | 1 | Protocol minor (`0`) |
| 2 | 1 | Firmware major |
| 3 | 1 | Firmware minor |
| 4 | 1 | Firmware patch |
| 5 | 1 | Node role |
| 6 | 1 | Current I2C address |
| 7 | 2 | Capability flags |
| 9 | 1 | EEPROM configuration-format version |
| 10 | 1 | CRC |

Capability masks are:

| Mask | Name |
|---:|---|
| `0x0001` | Laser sensing |
| `0x0002` | Button input |
| `0x0004` | Shared `FU` output |
| `0x0008` | Status LED |

Capabilities describe behavior for the commissioned role, rather than merely
listing peripherals present in the MCU.

### 6.2 `0x10 FAST_STATUS`

This is the only block required for routine laser-node polling during a run.
Read-only data before CRC:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | Boot counter |
| 2 | 2 | Monotonic event counter |
| 4 | 2 | Status flags |
| 6 | 1 | CRC |

The persistent boot counter increments once on every node boot. The controller
records it during preparation; any changed value during a run means the node
restarted and the run is aborted.

The event counter wraps modulo 65536. Reading never clears it. The controller
calculates modulo-16-bit differences from its stored baseline or previous
sample.

At 10 kHz, selecting and reading this block uses approximately 9 ms per node
before software overhead. A sequential poll of all 18 nodes therefore takes at
least about 162 ms and cannot meet a ten-polls-per-second full-inventory target
with the current transaction format. The measured stable node count and poll
duration must be used to revise the polling strategy or rate before closing bus
qualification.

### 6.3 `0x18 DIAGNOSTICS`

Read-only data before CRC:

| Offset | Size | Field | Notes |
|---:|---:|---|---|
| 0 | 2 | Raw ADC | Zero for button roles |
| 2 | 2 | Filtered ADC | Zero for button roles |
| 4 | 1 | Input state | Common input-state enum |
| 5 | 1 | Operating mode | `SETUP` or `GAME` |
| 6 | 1 | Last result | Common result code |
| 7 | 2 | Cooldown remaining | Milliseconds; zero for button roles |
| 9 | 1 | CRC | — |

### 6.4 Sensor configuration blocks

`ACTIVE_SENSOR_CONFIG` is read-only. `STAGED_SENSOR_CONFIG` is readable and
writable. Their data layouts are identical:

| Offset | Size | Field | Unit |
|---:|---:|---|---|
| 0 | 2 | Broken threshold | ADC counts |
| 2 | 2 | Hysteresis | ADC counts |
| 4 | 2 | Stable time | milliseconds |
| 6 | 2 | Cooldown | milliseconds |
| 8 | 1 | CRC | — |

Writes are accepted only in `SETUP`. The blocks are valid only for laser nodes,
except that an uncommissioned node may stage sensor configuration before
saving a staged `LASER` identity.

Validation ranges are:

| Field | Minimum | Maximum |
|---|---:|---:|
| Broken threshold | 0 | 1023 |
| Hysteresis | 0 | 1023, and not greater than threshold |
| Stable time | 0 ms | 1000 ms |
| Cooldown | 0 ms | 5000 ms |

The clear threshold is `broken_threshold - hysteresis`. A successful staged
write replaces the entire previous staged block but does not change active or
EEPROM configuration. The controller reads the staged block back and compares
it byte-for-byte before issuing `SAVE_CONFIG`.

### 6.5 `0x40 STAGED_IDENTITY`

Readable and writable data before CRC:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Desired normal address |
| 1 | 1 | Desired role |
| 2 | 1 | CRC |

Writes are accepted only from an uncommissioned node currently responding at
`0x08` in `SETUP` mode. Address must be `0x10`–`0x6F`; role must be `LASER`,
`START`, or `FINISH`. The controller must verify that the requested address is
unused because the node cannot detect an address collision itself.

The node does not change address or active identity until `SAVE_CONFIG`
succeeds.

## 7. Command mailbox

Commands use a fixed write to register `0xF0`. Bytes after the register pointer
are:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Command ID |
| 1 | 1 | Sequence |
| 2 | 4 | Argument bytes `arg0`–`arg3` |
| 6 | 1 | CRC over `F0`, command, sequence, and all arguments |

Unused argument bytes must be zero. The controller increments the sequence for
each new command to a node. It then reads `COMMAND_RESULT` until the returned
command and sequence match, or the transaction deadline expires.

The node caches the most recent validated command and result. Repeating the
identical command, sequence, arguments, and CRC returns the cached result
without repeating side effects. Reusing a sequence with different content
produces `SEQUENCE_CONFLICT`. Cache history is cleared at restart.

### 7.1 `0xF1 COMMAND_RESULT`

Read-only data before CRC:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Sequence |
| 1 | 1 | Command ID |
| 2 | 1 | Result code |
| 3 | 2 | Command-specific detail |
| 5 | 1 | CRC |

Before any command has completed, sequence and command are `0`, result is
`BUSY`, and detail is zero. Reading this register has no side effect except for
the explicitly deferred address-change and factory-reset behavior below.

### 7.2 Command IDs

| ID | Name | Arguments | Success detail |
|---:|---|---|---|
| `0x01` | `SET_MODE` | `arg0=mode`; others zero | Resulting mode |
| `0x02` | `SAVE_CONFIG` | All zero | Low byte address, high byte saved-record CRC |
| `0x03` | `RESET_COUNTER` | `arg0..1=preparation token`; others zero | Resulting counter (`0`) |
| `0x04` | `FACTORY_RESET` | ASCII `L`, `P`, `F`, `R` | Resulting address (`0x08`) |
| `0x05` | `IDENTIFY` | All zero | Identify duration in seconds (`4`) |
| `0x06` | `SET_BUTTON_LED` | `arg0=guidance state`; others zero | Applied guidance state |

#### `SET_MODE`

Changes volatile mode only. Entering `GAME` is rejected unless configuration
is valid and the node is commissioned. Node LED state indications, event
detection, and counters continue in both modes.

#### `SAVE_CONFIG`

Valid only in `SETUP`. At least one valid staged block must exist. The node
validates the complete proposed configuration, writes a versioned record with
CRC to EEPROM, reads it back, and only then replaces active configuration.
Failure leaves the previous active and persisted configuration unchanged.

If a staged identity changes the address, the node prepares a successful
result at its old address. It switches to the new address only after the
controller has completely read the matching `COMMAND_RESULT`. The controller
then waits at least 10 ms and reads `IDENTITY` at the new address. If the result
read was corrupted, recovery scans both old and new addresses.

#### `RESET_COUNTER`

Valid in `SETUP` for every role. Button nodes additionally accept it in `GAME`
so their visible game-mode brightness does not change during preparation. The
16-bit preparation token is generated by the controller and returned
implicitly through the matching command sequence; it distinguishes preparation
attempts in controller logs. The command atomically sets the event counter to
zero and clears `COUNTER_OVERFLOWED`. While in `GAME`, the controller may issue
it only in `WAIT_START` while Start acceptance is disabled. It must never reset
counters in `WAIT_FINISH`.

Before every run, the controller resets every node, reads every `FAST_STATUS`
block back, and refuses to enable Start unless every event counter is zero. It
stores the boot counters and zero baselines. During preparation, only laser
nodes temporarily enter `SETUP`; button nodes remain in `GAME`, preventing
their LED brightness from flashing between players or after the three-second
clear interval.

#### `FACTORY_RESET`

Valid only in `SETUP`. The four arguments must be ASCII bytes `L`, `P`, `F`,
`R` (`4C 50 46 52`). This protects against an accidental write; it is not a
security mechanism.

The node prepares a successful result before clearing EEPROM. It erases its
identity/configuration and restarts at `0x08` only after the controller has
completely read the matching result. The controller waits at least 20 ms before
probing `0x08`.

#### `IDENTIFY`

Valid only in `SETUP`. The node drives its status LED with the fast Identify
pattern for four seconds, temporarily overriding its other LED state. The
command does not change configuration or event detection. Retrying the same
command and sequence returns the cached result without restarting the timer; a
new command sequence starts a new four-second interval.

#### `SET_BUTTON_LED`

Valid only in `GAME` and only for commissioned `START` and `FINISH` nodes.
The other arguments must be zero. `arg0` has these values:

| Value | Name | Released-button indication |
|---:|---|---|
| 0 | `STANDBY` | Steady 5% glow using approximately 1.25 kHz PWM |
| 1 | `READY` | Steady on; the player may press this button |
| 2 | `BLOCKED` | Regular 1 Hz blink with equal on/off phases |

The setting is volatile and resets to `STANDBY` whenever the node enters
`SETUP` or restarts. Setup mode itself displays a commissioned button steadily
on. The immediate off-while-pressed and long-press blink indications retain
priority over game guidance.

## 8. Transactions, retries, and timing

The initial deadline for one register write or combined register read is 10 ms.
For a mailbox command, each individual register transaction uses that deadline;
the total command deadline is initially 50 ms, except EEPROM commands, which
use 100 ms. These values must be measured and adjusted on the full bus.

After a timeout, NACK, short transfer, CRC error, or invalid content, the
controller retries a transaction at most twice:

- Read registers are simply selected and read again.
- Staged-block writes repeat the complete identical block and are read back.
- Mailbox commands repeat the identical sequence and bytes, relying on the
  command cache to prevent repeated side effects.

A node that remains unavailable after two retries is faulted. The controller
does not continuously retry a failed node during a run.

## 9. Event behavior

### 9.1 Laser nodes

Laser sampling, filtering, hysteresis, stable time, and cooldown operate
locally. An accepted clear-to-broken transition increments the 16-bit event
counter once. A continuously broken beam does not count again. The beam must
return to clear for the stable time and cooldown must expire before another
interruption can be counted.

The controller reads `FAST_STATUS` from every laser node at least ten times per
second during a run. Different laser nodes count independently, including
simultaneous interruptions.

### 9.2 Start and finish nodes

Button nodes use immediate press detection followed by a release debounce. The
first inactive-to-active edge on an armed button immediately increments that
node's event counter and generates one 10 ms active-low pulse on `FU`; the node
does not wait 50 ms before reporting the press.

After that first edge, the button is disarmed and all further active/inactive
edges are ignored. It is re-armed only after the raw input has remained
continuously released for the initial 50 ms release-debounce interval. The
pulse duration is independent of how long the button remains pressed. Holding
a button, press bounce, or release bounce therefore cannot create multiple
events. Pulse timing must be non-blocking; firmware must not wait in a delay
loop while `FU` is asserted.

Immediate acceptance avoids adding the debounce interval to the measured start
or finish time. Its tradeoff is that a very short electrical disturbance can
be accepted as a press, so this behavior must be validated on the installed
button wiring.

Multiple button nodes share `FU` using open-drain outputs. Overlapping pulses
appear as one continuous low level, so event counters remain authoritative:
after every falling edge, the controller reads both button-node counters. The
controller also includes both button nodes in its regular status polling so a
counter change cannot remain unnoticed if two pulses overlap completely.

The Pico timestamps the falling edge in its GPIO interrupt handler. Outside
the handler, it reads `FAST_STATUS` from both button nodes and compares their
counters with the stored baselines to identify the source. If both advanced,
both events use the same captured edge timestamp and the controller state
machine decides whether that combination is valid. A counter advance found by
periodic polling without a corresponding captured edge is logged as an event
line/timing fault rather than silently assigning an inaccurate timestamp.

The controller accepts a Start event only while its game state is
`WAIT_START` and Start readiness is enabled. It accepts a Finish event only in
`WAIT_FINISH`. Other button events are logged and ignored without changing the
game state. Laser counter changes contribute to the attempt only in
`WAIT_FINISH`; in `WAIT_START`, the first broken beam disables Start and starts
a retriggerable three-second interval. The interval is held at zero while any
beam remains broken and is reset by every later interruption. Start is enabled
only after all beams have remained clear for the complete interval and counter
reset has been repeated successfully.

## 10. Restart and fault behavior

The controller stores every node's boot counter during preparation. Counter
comparison is modulo 65536; any changed value, rather than ordering, indicates
a restart.

During a run, any of the following aborts the run:

- a required node remains unavailable after retries;
- register CRC, length, role, or protocol identity remains invalid;
- a node boot counter differs from its preparation value;
- a node reports invalid configuration or another unrecoverable status; or
- the discovered node inventory changes.

Outside a run, the same conditions prevent arming and put the controller into
its bus/node fault state until a successful rescan and preparation check.

Malformed or invalid writes never modify active or persisted configuration.
When enough of a bad write is available to classify it, the node records the
corresponding result in `DIAGNOSTICS` and sets `LAST_OPERATION_ERROR`.

## 11. Versioning

Protocol 1.1 adds `SET_BUTTON_LED`. A 1.1 controller may still inspect a 1.0
node, but game-mode preparation requires Start and Finish nodes that implement
the new command.

Protocol major version 1 is the byte-level contract in this document. A major
version change may alter existing addresses or layouts and is incompatible. A
minor version may allocate reserved registers, capability bits, status bits,
commands, or enum values without changing defined version-1 behavior.

A controller accepts protocol major `1`, ignores unknown capability/status
bits, and does not use unadvertised optional capabilities. Firmware version and
EEPROM configuration-format version are independent of protocol version.

## 12. Implementation and verification checklist

- Create a shared header containing register addresses, command IDs, masks,
  limits, and block layouts, with compile-time size assertions on both targets.
- Implement one CRC routine and verify shared test vectors on both targets.
- Test coherent snapshots while ADC values and counters change during reads.
- Test every writable block with valid, truncated, oversized, bad-CRC,
  wrong-mode, wrong-role, and boundary-value writes.
- Test command timeouts, lost result reads, duplicate sequences, and sequence
  conflicts without duplicated side effects.
- Test counter reset and zero verification on every node before each run.
- Test EEPROM interruption, invalid stored CRC, commissioning, address-change
  recovery, and factory reset.
- Test node power-cycle detection and 16-bit event-counter wrap during a run.
- Measure timing, retries, rise times, and achieved polling rate using the
  normal 8–12-node inventory on a representative branched harness;
  separately confirm that an inventory of up to 18 nodes is accepted.
