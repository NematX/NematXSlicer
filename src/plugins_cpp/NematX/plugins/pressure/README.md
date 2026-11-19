# Analytical motion phases

`PressureKinematics.hpp` accepts an already normalized `Motion` and returns an
owned `Profile` with at most three phases: acceleration, steady motion, and
deceleration. Only the first `count` array entries are used. All returned values
remain valid independently of the input. No configuration, G-code parsing or
writing, normalization, axis projection, or runtime integration is performed.

The caller supplies distance D in mm, nominal speed limit V and endpoint speeds
v0/v1 in mm/s, and positive acceleration/deceleration magnitudes a/b in mm/s².
Acceleration and deceleration are caller-supplied model data; this solver defines
no policy for selecting them.
These quantities must use the same dominant-axis model coordinate. The historical
coordinate uses max(|dX|, |dY|, |dZ|, |dE|); selecting and normalizing that
coordinate belongs to the caller, not this solver. Model durations are not a
prediction of a controller's actual timing.

All inputs must be finite, D/V/a/b strictly positive, and endpoints within [0,V].
An impossible endpoint change throws `std::invalid_argument`. Its minimum
distance is |v1-v0| (v1+v0)/(2a) when accelerating, or the same expression with b
when decelerating. A positive deficit no larger than
8 × double epsilon × max(D, minimum distance) is treated as a rounded monotone
boundary with the original endpoints and length D. Endpoints above V are rejected
even by one representable step. Nonrepresentable phase results also throw.

At V, acceleration/deceleration distances are (V-v0)(V+v0)/(2a) and
(V-v1)(V+v1)/(2b). Any remaining distance is steady motion, even if it is a small
fraction of D: D=10.05, V=v0=10, v1=0, b=5 yields a 0.05 mm steady interval
(0.005 s) followed by 10 mm deceleration (2 s).
Otherwise the triangular peak satisfies
vp²=(2abD+b v0²+a v1²)/(a+b); the implementation solves from the higher endpoint
to avoid subtraction of nearly equal squared speeds. Distances need not split
equally, even with equal acceleration magnitudes when endpoint speeds differ.

Adjacent phases share exactly the same speed value; original endpoints are kept
exactly. Every emitted distance and duration is positive, and duration is
2d/(speed_start+speed_end). Empty phases are omitted. If a peak speed change is
not representable, its distance is included in the neighboring phase, or in a
steady phase when both nonzero endpoints coincide. This is floating-point
resolution handling, not coordinate quantization. No pressure coefficient,
extrusion correction, junction calculation, or motion history is involved.

## Printing movement planner

`PressurePlanner.hpp` provides a pure normalizer and a planner for a cycle of
already classified printing movements. Normalize signed XYZ deltas and a
nonnegative E delta, then push each owned `Move` into one `Planner`. A spatial
movement with E=0 is valid; E-only movements are rejected. The caller handles
travel, retraction, tools and cycle boundaries. `source_index` is an opaque
numeric ID, not a line parser. Returned `Publication` values own stable moves
and profiles; `finish()` returns the remainder with terminal speed zero and
resets the cycle. Empty or repeated finish is empty; reuse starts at zero.

The model coordinate is D=max(|dX|,|dY|,|dZ|,|dE|), in mm. The separate reach
Dxyz=max(|dX|,|dY|,|dZ|) controls buffering, not Euclidean path length. Given
feed F>0, positive acceleration A and axis limits vi/ai, the normalizer computes
T=max(D/F, |di|/vi) and V=D/T, and a=min(A, ai D/|di|), ignoring zero axes.
Nonpositive axis limits fall back to 100000. A positive maximum extruding
acceleration caps XYZ accelerations after fallback, but not E acceleration.
Passing all limits and that cap as zero implements the caller's ignored-limits
mapping; leaving a positive extrusion cap is not equivalent. Each move stores
its resulting acceleration, so later changes of A cannot affect earlier moves.
Acceleration and braking both use this same projected a, not separate policies.
For D=20, V=v0=10, v1=0 and projected a=4, the profile has 7.5 mm steady motion
and 12.5 mm braking. Global projection also makes dX=dY=10, F=10, vx=2, vy=5
produce V=2 rather than a result dependent on axis iteration order.

Only XY directions are rounded symmetrically to integer nanometres (1e-6 mm).
Distances, feed, extrusion and phases are not quantized here. The interior angle
between the reversed preceding ray and the next ray gives factor zero below
90 degrees, otherwise (angle-90)/90. Straight motion gives exactly one, a
45-degree turn gives about one half, and a right angle, reversal or zero XY ray
gives zero. Negation uses the already rounded integers. Coordinate differences
representing repeated (.1,.2) rays therefore give exactly straight junctions.
The cap stored once at each arrival is factor times min(previous attained end
speed, new V). A previous attained speed of 20 with nominal 100 and factor .5
gives 10, not 20. The factor is not reapplied during later passes.

Hard junction caps remain distinct from effective speeds. Each recalculation
starts from the caps, propagates braking backward using v0<=sqrt(v1²+2aD), then
acceleration forward using v1<=sqrt(v0²+2aD). The open tail starts at its V cap;
closure uses zero. Shared endpoint doubles preserve exact continuity. Each
newly computed reachable speed is rounded downward with `nextafter` until
(result-base)(result+base)/(2a)<=D, using the phase solver's distance expression.
Both passes and the stopped envelope share that calculation; the solver and
its input tolerance are unchanged. This prevents an upward-rounded root from
making an ordinary movement fail phase solving. Stored caps are not permanently
clipped to the effective speeds. In exact arithmetic, adding a cap no higher
than the previous attained endpoint cannot increase earlier effective endpoints.
The rounded bound is not proven monotone in its base speed for every floating-
point input, so universal bit-for-bit continuation stability is not claimed.

Publication considers whole source moves only while both retained count>100
and retained Dxyz sum>100 mm. It computes a separate backward envelope with
tail zero, with the same operations as closure. A candidate is published only
if its open endpoint is exactly <= the stopping envelope at that junction.
That known suffix can already brake to zero. This supplies the stopping proof
in exact arithmetic; remaining floating-point incompatibilities are reported
as `std::logic_error`, never by lowering a published entry. Successful closure
for every possible numeric continuation is not guaranteed. The stopped copy
never changes open speeds or caps.
After each removal the published endpoint becomes the suffix's fixed entry and
the retained reach is resummed. If the longest safe prefix still leaves both
thresholds exceeded, `horizon_insufficient` is true; extra suffix retention is
allowed without forced publication. Counts refer to source printing movements,
not phases. No anticipation crosses a caller's cycle boundary.

Invalid inputs throw `std::invalid_argument` before insertion; violated internal
reachability throws `std::logic_error` instead of changing a published speed.
After `logic_error` from `push`, the caller must abandon the instance and fail
processing; the retained cycle is not rolled back.
This is an approximate dominant-axis and angular motion model, not a controller
simulation. The standalone Debug and Release tests exercise numeric planning,
not pressure export. No F commands, G-code text, runtime pressure K, extrusion
correction, Reader, configuration or firmware integration is implemented here.

## Quantized filament positions

`PressureExtrusion.hpp` provides `Extrusion`, an independent value state for a
caller that already has classified printing phases. Construct it with fixed E
decimal precision (0 through 9) and a nominal origin in filament mm. Pass each
phase's **cumulative** nominal endpoint N, model endpoint speed v in mm/s, and
that movement's copied pressure factor K in seconds to `advance`. Nominal
endpoints must be nondecreasing; a zero increment is valid. The caller supplies
only phases represented by actual spatial movements, not synthetic E-only lines.

The algebraic charge is C=K*v in filament mm. This is the dominant-axis model
speed used by the planner, not feed in mm/min or filament speed. At model speed
100 mm/s, filament speed 4 mm/s and K=.02 s, C is 2 mm, not .08 mm. Absolute and
relative output use the same law. Existing absolute calibrations based on filament
speed have no universal conversion, and none is performed.

The state stores N, charge C and the last quantized output P. Each call returns
an owned result with P=Q(N+C), delta=P-P_previous, nominal_delta=N-N_previous,
pressure=C and pressure_delta=C-C_previous. Q is the public SDK formatter's E
quantizer, with ties rounded away from zero. Only digits are stored; a local
formatter supplies the inline quantization without a host library dependency.
Intermediate nominal phase endpoints remain doubles until adding charge and
quantizing the combined target; the final phase must use the exact source
endpoint. Nominal source values are the session's E words, which may already
be quantized. This state cannot recover private host volume absent from them.

Select **either** the returned position for absolute output **or** the delta for
relative output. They are final E values, not new nominal extrusion, and must not
be passed through `ExtrusionAxisState::extrude`. The nominal session has already
advanced once before this calculation. A negative delta caused by releasing
charge is not a semantic retraction and creates or repays no retraction debt.
This numeric component has no access to that accounting or to statistics.

Relative working sums start at zero and persist across layers and closed cycles.
An external machine reset to -1.25 does not make -1.25 the origin of that sum.
A relative G92 or SET_POSITION does not reset that independent working sum. A true
E/XYZ/unit mode change rebases the appropriate working frame after charge is zero.
Absolute state instead starts at the
current nominal coordinate, including off-grid origins such as 5.125 or -1.25.
Its baseline is Q(origin); returned deltas are measured from that baseline, not
the exact physical displacement from an off-grid firmware reset. Absolute output
is the returned position Q(N+C).

An endpoint at speed zero closes charge exactly, so P=Q(N). K=0 also closes it
even at nonzero speed, returning any preceding charge through the difference.
Closing a layer does not reset N or P: the same state continues the next cycle.
Recreating an absolute state at the current nominal coordinate is equivalent at
a closed boundary; recreating it at zero while keeping the cumulative N is not.
`rebase` emits nothing and is allowed only at C==0; the caller must already have
established the new coordinate frame. Open rebase throws `std::logic_error`.
Invalid precision, nonfinite N/origin/K/speed, negative K/speed or nonrepresentable
arithmetic throws `std::invalid_argument` before changing existing state.

The supported numeric domain is |corrected position|*10^digits <= 1e14
(100000 mm at E9). Within this domain differences between quantized positions
are checked to encode back to the expected integer ticks with the real SDK
quantizer. There is no hard clamp or extra setting. A script with nominal E words
finer than configured output precision can move a relative working sum off-grid;
even at K=0 some emitted deltas may then differ by one tick from independently
rounded source deltas. This numeric behavior alone does not qualify real exports.
Only MSVC with `/fp:precise` is qualified by the standalone Debug/Release tests;
other compilers are unqualified.

The standalone extrusion tests exercise literal integer-tick oracles, signed
rounding ties, pressure release, independent states and persistent closed cycles.
No geometry merging, G-code writing, Reader adaptation, firmware/plugin API or
runtime pressure export is connected or qualified by this component.

## Spatial projection before extrusion

`PressureGeometry.hpp` projects one valid `PlannedMove` and its `SourceGeometry`
onto the configured XYZ grid (0..9 digits), using only the inline SDK quantizer.
Absolute XYZ and cumulative nominal filament endpoints are doubles in mm from
the source words, which may already be quantized. The interpolation fraction is
the phase prefix divided by the dominant model distance, not XYZ path length.
Interpolated coordinates and nominal values are bounded by their source endpoints
to prevent floating-point overshoot. The last endpoint uses the exact source end
before XYZ quantization; nominal filament is never quantized by this component.

The owned result partitions the original phases in order. Collapsed head or middle
phases join the next representable group; a collapsed tail joins the last group
of this same source before publication. `first_phase` and `phase_count` identify
the partition. `Merged` means at least two phases: distance and duration are sums,
not a new constant-acceleration phase or a phase feed command. Resolution loss is
identified by a merged group or `EntireSourceCollapsed` status.

Check status first. `Representable` contains one to three nonempty spatial groups.
Only these final groups may be passed to `Extrusion::advance`, using nominal end,
endpoint speed and the K copied for the source. Discarded cuts never advance E.
A merged tail at speed zero closes charge at the final nominal endpoint. At fixed
K the emitted charge differences sum to K*last speed minus the incoming emitted
charge. This equals K*(last speed-first speed) when that incoming charge is K*first speed.
After a K change, `pressure_delta` instead releases the actual preceding charge;
it need not equal current K times the group's speed difference. Negative output
E is allowed and is not a semantic retraction or new nominal accounting.

`EntireSourceCollapsed` contains one **non-emittable** aggregate retaining all
phases, nominal endpoints, distance, duration and endpoint speeds. Do not call
`Extrusion::advance` on it. The component makes no inter-source choice across
different K, feed, tools or events and creates no compensating E-only movement.
Generated XYZ sources already have an output-grid displacement; finer script
sources can collapse entirely. Preserving their lines, material and charge through
stream barriers is not resolved or qualified here. The model origin is Q(start),
not an exact preservation of off-grid script text.

The caller guarantees valid planner/profile data (1..3 positive phases), coherent
source geometry and nondecreasing nominal endpoints. Invalid precision, nonfinite
source values or nonrepresentable required arithmetic throw `invalid_argument`
without modifying borrowed data or publishing a partial result. Standalone tests
qualify ordinary machine-scale coordinates and binary rounding boundaries with
MSVC C++17 `/fp:precise` only; other compilers are unqualified. No Reader, text
emission, classification, configuration or runtime pressure export is connected
to this geometry component.

## Owned Reader inputs

`PressureReader.hpp` provides `read_fragment(service, prior, text)`, a synchronous
client of the SDK Reader service. It always requests exactly
`gcode.reader.nematx`, with no default or fallback. The service and its borrowed
orchestrator/Print context must live throughout the call. The caller supplies one
complete Reader input, never a command split between calls. Empty text and a
complete last command without LF are valid; there is no pending-line buffer.

The owned `ReaderFragment` contains `initial`, ordered `events` and `final`.
Initial is exactly prior with only `LINE_NUMBER` set to zero. All coordinates,
tools, modes, Reader identity and opaque private items persist. Line numbers are
local to this input, not global diagnostic indices. Each event copies its kind
and every item during the SDK's validated observer callback, including local
markers and diagnostics. Multiple events on one line retain their order.
No provider pointer or source text view survives the call.

Replay by forming temporary raw events from the owned item vectors and applying
them to a copy of initial. No complete tracker is stored per event. Final is the
authoritative state returned by the SDK: its private items and line cursor may
differ from event replay. Pass final as prior for the next complete input.
Copy/move destinations retain valid tracker raw views; moved-from trackers may
only be destroyed or reassigned, not used as complete Reader states.

Unavailable/wrong-step providers, execution failure, cancellation, malformed
events and missing final states propagate as SDK errors. No partial result is
returned and prior is unchanged. The observer only copies under the service's
nonrecursive mutex; it never invokes another read. This helper supplies no retry,
global cancellation recovery or independent synchronization protocol.

Standalone Debug/Release tests link only this client and three local orchestrator
C stubs. A scripted provider drives the real SDK pull and validation callbacks;
it does not parse or imitate NematX commands. These tests qualify ownership,
ordering, state propagation and error isolation of the client protocol, not a
real Reader, runtime activation, pressure interpretation or export behavior.

## Original numeric tokens

`PressureTokens.hpp` supplies `read_parameter(line, literal_letter)` for a real
SDK `GCodeLine`. It skips the first word (normally the command), queries the exact
case-sensitive letter and selects its first duplicate before the semicolon
comment. Absence returns `nullopt`; a present invalid number throws
`std::invalid_argument`.
The SDK's public float parser validates grammar, delimiters and finiteness once.
The helper then decodes the original text directly into double with the same
`fast_float` grammar, never using the float as its numeric result. Thus 16777217
and ordinary 12.3456789 retain double precision; 1e-50 remains 1e-50 even though
the SDK float is zero. Underflow such as 1e-400 is accepted as zero, while 1e39
is rejected by SDK float validation even though double could represent it.

The owned result contains only value, offset and length. Its numeric byte range
excludes the letter and following spaces/tabs and indexes `line.raw()` without
CR/LF. The scalars survive source destruction, but applying a range requires the
same original unchanged text. Fragment offsets and edit ordering belong to the
caller. No production rewriting is provided.

This is lexical access, not Reader authority: known state, modes, axes, tools and
original-letter mapping remain the caller's responsibility. Both `G1 x1 X2` and
`G1 G90 x1 X2` yield 2 for literal X and 1 for literal x here. The actual Reader
path may select different original words after dialect translation; global
case-insensitive first-match or uppercase-only mapping is not sufficient.
Accepted exponent syntax does not establish NematX command support. Bracketed
NematX resets are not decoded by this helper. Standalone Debug/Release tests use
the unchanged SDK lexical parser, not a real Reader, and qualify neither active
pressure export nor any mode interpretation, classification or firmware behavior.

## Original source lines and event ranges

`PressureSource.hpp` joins owned original text with the `ReaderFragment` from
exactly the same `read_fragment` call. Read first, then pass text and Reader result
to `index_fragment` in a separate instruction; do not combine a move of text with
a read of text in one argument list. Events must already be SDK-validated.

The result owns text, Reader data and `SourceLine` records. Each record contains
only byte offset/length and first-event/count. Its vector index is the local
physical line number. LF alone frames records; terminators, CR, NUL, spaces,
comments and N prefixes remain byte-for-byte intact. Empty text has no records;
a terminal LF adds no phantom record. Lines without events retain the current
event cursor. Multiple events on one line keep their original order and items.

The NematX final cursor must equal the number of LF bytes, not the record count:
a complete final command without LF does not advance it. Event lines must be
in bounds and nondecreasing. Violations throw `std::invalid_argument` with no
partial return. These checks do not establish identity between texts with the
same LF count and do not replace the same-read precondition. Initial, final and
events are preserved without requiring replay to reproduce authoritative final.

The Reader may remove an N prefix according to its grammar and translate axes
or modes. Its semantics cannot be inferred from these raw ranges alone. Lexical
`line.raw()` can end at CR or NUL before this LF-delimited record ends; preservation
here assigns neither neutral semantics nor a refusal policy to those bytes.
Implicit copies and moves keep destination storage independent, including SDK
state views. Moved-from Reader states are only for destruction or reassignment.

Standalone Debug/Release tests use manual events checked through public SDK
`GCodeStateTracker::apply`, with no Reader service, C stubs, parser implementation
or host archive. This pure linear join performs no nominal calculation, command
classification, axis mapping, rewriting, configuration or pressure runtime.
It does not qualify active NematX export or resolve cross-source geometry loss.

## Persistent nominal double coordinates

`PressureNominal.hpp` follows the same `SourceFragment` with an owned
`NominalState`: known XYZ and per-target E positions in mm, plus modal F in
mm/min. Start empty at the beginning of an export and carry `final` across
fragments and layers, including text outside layers. Pass the exact axis letters
resolved for the Reader's print context. Same read, same export and corresponding
prior remain preconditions; matching coordinate keys cannot establish identity.

The known XYZ/E-target/F keys must exactly match Reader initial and final states.
Either mismatch throws `std::invalid_argument`, without a partial result or a
fallback to Reader float values. Values themselves are not compared. Private
state and final line cursor are not compared with event replay. A local SDK
tracker applies all events in order to expose proven pre-motion flags; unknown
flags remain absent, even where Reader defaults permit numeric tracking.

Each linear or arc event retains double endpoints, E target/start/end/delta,
modal F, selected X/Y/Z/E/F numeric byte ranges and pre-motion modes/tool. Arc
endpoints are followed only; this provides no pressure support for arcs. Reader
items select coordinate changes and invalidations. Relative E with unknown
origin can prove its source delta without proving an endpoint. Absent E proves
zero delta, while an SDK-invalid E does not. A semantic E target does not by
itself prove an active tool. Invalid reset coordinates and invalid F omitted by
the Reader preserve their previous values.

The original LF record is stripped with NematX's existing line-number function
before SDK lexical CR/NUL truncation. Translation runs once with pre-line modes.
Direct commands use literal SDK selection; translated geometry uses the exact
standard text, then maps its selected word ordinal back past only exact G90/G91
words. Numeric bytes must match, or `std::logic_error` is raised. Source letters
retain their case. Only three scalars (offset, length, letter) survive per token;
the unchanged source text is required to apply those ranges. Bytes discarded by
lexing are not qualified as neutral or safe to emit.

G28 uses the Reader's selected XYZ items as exact zeros; G92 reads accepted
source doubles with unit scaling. Bracket resets use the Reader's existing
double E value without unit scaling. Duplicate selection follows the Reader's
first effective match, not a claimed controller or writer policy. Original text
still contains every duplicate word.

The standalone `pressure_nominal` test uses manual SDK-validated deltas, the real
SDK parser and existing NematX syntax, without a Reader service, SDK archive or
host library. It qualifies only this nominal channel, not classification,
barriers, pressure calculation, correction, rewriting or active export.

## Line classification and cycle boundaries

`PressureClassification.hpp` joins the same source and nominal read with caller
resolved `ClassificationContext` and per-row `MoveHint` values. Hints are empty
or exactly the physical row count. Every generated movement needs its hint;
scripts may use None. Text cannot prove completeness of this caller obligation.
The caller resolves `enabled_tools` from configured K>0 or F09, independently of
the global pressure override, and supplies resolved `use_volumetric_e` as
`config_volumetric`. An empty tool vector is not a neutral-tool declaration.

Each returned row contains `close_before` and, only for eligible printing, an
index into `NominalFragment.motions`. These are not Reader event indices. The
result owns only scalars and indices, not text, coordinates or borrowed views.
`close_before` closes the previous cycle before processing this row once; a row
with a tool change and an eligible print can itself begin the next cycle.
Carry `final` into the next fragment of the same export. Copying prior preserves
it on failure; moving intentionally transfers it. No partial fragment is returned.
Input shape errors throw `invalid_argument` or `logic_error`; active unsupported
printing throws `ClassificationError` with local zero-based row, tool and cause.

The cycle snapshot records proven pre-motion modes of the last accepted print.
Normal inline G90/G91 does not compare its transient post-motion XYZ mode with
that snapshot. Rows without motion and linear NOOPs with no E and no effective
XYZ displacement compare only net updated mode/tool slots across the whole row,
including updates after a motion event. Residual modes alone do not close them;
restoring absolute XYZ or millimetres is transparent when it matches the cycle.
Physical travels, wipes, retractions, E-only motion, resets, relevant invalidation,
unknown commands and effective mode/tool changes close the previous cycle before
the row. F-only motion without such an update or another barrier is transparent.
Metadata-only invalidation and parameter diagnostics
alone are transparent. Actual Reader-unknown M204 closes; its comment does not.

Wipe scope requires an exact comment-only WIPE_START/WIPE_END after stripping the
N prefix and SDK CR/NUL lexical stops, with only surrounding spaces/tabs removed.
Scope survives TYPE changes, fragments and time outside layers until WIPE_END.
The known Reader wipe role and Wipe hints independently identify nonprinting.
Travel hints dominate positive E (generated fixed-E travel is possible). No role
proof is needed for printing. Nonzero script XYZ is classified before quantizing;
stationary XYZ plus E and host-elided generated XYZ are E-only at this input.
The host can remove generated XYZ before encode_move; that lost geometry cannot
be reconstructed here. Existing E is preserved, with no compensating E-only move.

Active printing requires the proven tool/target, absolute XYZ, millimetres, finite
XYZ endpoints, positive finite F and exact nonnegative finite E delta. Absolute E
also needs finite external start/end. Relative E does not need those external
coordinates: its independent working sum starts at zero in the extrusion model.
No absolute E key or zero origin is invented. An immediately preceding same-row
E-only invalidation for that target is exempt from closing only when that linear
move is accepted as a relative print with a valid SDK E token and exact delta.
Other barriers on that row still close. Hinted spatial printing with absent E is
valid only after tool and the effective E mode have actually been proven, with
the numeric proofs required by that mode: relative delta alone or absolute
start/end/delta. A hint proves neither the tool nor the E mode.

Arcs reported by the Reader are refused for an active tool inside layers, even
with Travel/Wipe hints. Neutral tools and text outside layers are unchanged.
Already chorded input is linear, not new arc support. Outside layers, no print
proofs are demanded, but wipe/role and volume controls still follow the stream.
The cycle snapshot is cleared outside layers; mathematical C=0, layer/print
flushes and E rebasing belong to the caller, not this classifier.

G4/G04, bare M109/M129/M190, Reader temperature waits and pause/color markers
close before their unchanged row. This differs from transparent legacy pauses:
at K=.02 s and model speed 100 mm/s, the caller must return the 2 mm charge before
the pause and restart from zero. There is no decompression model, extra delay,
precharge or phase F command. Nonblocking temperatures, fans, SEAM, CUSTOM_GCODE,
objects, progress and TYPE remain transparent unless another barrier is present.

Configured volumetric extrusion, a known-true Reader volume item, or any recognized
uppercase numeric M200 command makes active candidate printing unsupported.
The M200 guard accepts leading zeros and assigned syntax without decoding D0/D1:
even M200 D0 latches distrust. Only a new export clears it, not a layer, reset,
tool, unit change or later false/unknown Reader item. Postambles without another
active print are not rejected. Generic unknown commands do not set this latch;
printing can resume after required coordinates and modes are proven again.
Opaque macros or controller configuration changing volume cannot be detected:
disabling those hidden changes is a user prerequisite. The Reader is not assumed
to know firmware configuration. Completely disabled runtime functions must bypass
the engine; that integration is not supplied here.

Standalone `pressure_classification` checks manual events through public SDK
`apply`, then obtains numeric inputs from the real nominal component and checks
literal classification oracles. Existing SDK parsing and NematX syntax verify
representative lexical/action assumptions. Structural adversarial inputs are not
claims about an actual provider. No Reader provider, host archive, configuration
lookup, pressure planner, correction, rewriting, producer hints or export hook is
connected. Real Reader/hint/configuration integration and active exports remain
unqualified.

## Prepared pressure text

`plugins/pressure/src/PressureRewrite.hpp` replaces one already eligible printing
row with 1–3 prepared spatial groups. It does not run a pressure engine, look up
configuration or connect to the firmware/export pipeline. The caller must supply
matching Source/Nominal data from one read/export, proven modes and active tool,
resolved extrusion letters, representable geometry and aligned extrusion results.
An entirely collapsed source must be resolved by the caller before this API; it
is not dropped or replaced with an E-only compensator.
After the caller has arranged transfer of that source's material, `rewrite_residue`
removes its complete XYZ/extrusion words without adding a pressure annotation.
It preserves the command, N prefix and remaining bytes when F or a useful modal
effect remains; otherwise it returns only the source comment with its original
ending, or an empty string when there is no comment.
It compares the original and residual mode/tool effects from the true Reader
state before the line and throws `RewriteError` with cause `LostModalEffect` if
removing words loses one.

The first group preserves the command, line-number prefix, spacing, all numeric
F words and source comment, editing only selected numeric XYZ/E spans and adding
an active-tool extrusion word when required. Later groups contain only the exact
command, present XYZ axes, required E and inline modes. No phase feedrate is sent.
Existing generic E remains generic, including direct G01/M83 syntax; axis letters
retain the Reader's extrusion representation. VT/FF separators and case folding
follow the translated path only; the first row retains its source extrusion
letter, while constructed generic extrusion words use uppercase E for SDK parsing.
LF, CRLF and a final row without LF are supported. Eligible rows reject duplicate
effective XYZ/E words, conflicting XYZ modes, unproven words, NUL and CR outside a
terminal CRLF. Errors own the local zero-based row, proven tool and cause; no
replacement prefix escapes on failure.
This policy does not classify or newly reject travels, wipes or disabled tools.
Global all-K-zero/F09-off identity belongs to the caller's no-engine path, not a
per-source bypass here: current K=0 may still release a preceding charge.

F09 adds one inline annotation to **every emitted group**, after any original
comment on the first group. `;NX_PRESSURE ` is a reserved annotation prefix:

```
;NX_PRESSURE v=1 tool=0 phase=accel v0_mm_s=0 v1_mm_s=2 e0_mm_s=0 e1_mm_s=0.2 dt_s=2 k_s=0.1 de_nom_mm=0.2 de_pressure_mm=0.2
```

All fields are mandatory and ordered as shown. `phase` is `accel`, `steady`,
`decel` or `merged`; merged denotes precision-driven spatial fusion, not a new
physical phase. Velocities and durations describe the dominant-axis model, not
machine timing or speed commands. E velocities are nominal filament velocities;
deltas are analytical nominal/pressure values, not differences reconstructed from
rounded E text. Numbers use locale-independent shortest round-trip precision,
independent of axis digits. Normal `gcode_no_comment`/OutputFilter removal applies;
annotations are not protected, and removing them does not remove correction.

The isolated `pressure_rewrite` and `pressure_rewrite_annotations` CTest tests use
manual SDK-validated events, real nominal/classification/math components and the
SDK parser/formatter. Python parses actual executable output in memory. These are
text/codec checks, not qualification of a Reader provider or an export/runtime.
Hidden controller/macros' volumetric modes and pressure advance must be disabled
by the user; this text helper does not detect them. F03 and the preview's nominal
material reconstruction are not changed or qualified here.

## Persistent model controls

`PressureControls.hpp` reads one complete physical source record at a time into
an owned `ControlState` snapshot: optional pressure override K in seconds and
model acceleration in mm/s². The caller initializes a fresh state per export,
with no override and finite positive acceleration resolved from the legacy X
limit (nonpositive falls back to 100000), capped by a positive maximum extruding
acceleration. Carry returned `next` across fragments, layers and tool changes;
copy settings when capturing a movement so later updates are not retroactive.
Configuration lookup, activation per tool and effective K selection are not
provided. An override alone must not awaken a neutral tool; an enabled F09-only
tool can use a positive override. The all-neutral no-engine path belongs to the
caller and is not qualified here.

Recognition is case-sensitive at byte zero only: `; PRESSURE_FACTOR=` and
`;M204 S`. There is no indentation or line-number normalization. One terminal LF
or CRLF is removed logically; a last record without LF is valid. Fields permit
only surrounding spaces/tabs and complete decimal/scientific C-locale doubles,
including one optional plus sign. K must be finite and nonnegative; empty or
blank clears it back to configuration, while zero remains an explicit override.
Exact negative-zero forms normalize to positive zero. Negative nonzero mantissas
are rejected even if double underflow produces zero; positive underflow is
accepted as zero. Invalid K throws `ControlError` with local zero-based physical
row and cause, without changing prior or returning a partial state.

The model comment `;M204 S...` accepts only finite positive acceleration.
Invalid fields preserve both prior fields and return an owned `ControlIssue`
with row and cause. The caller must route this diagnostic through an existing
export diagnostic channel; this component supplies no logger or GUI. Actual
`M204` commands and other comment forms are opaque here; their Reader barrier
and invalidation behavior is not masked or changed. Commented acceleration is
a model assumption, not a command to the controller. Control updates alone
do not close cycles in this component.

Unlike the legacy layer-only control scan, the intended caller reads controls
from the preamble onward whenever an engine exists; invalid K can therefore
fail before the first layer. Double decoding avoids legacy `strtof` rounding
(K=.02 at model speed 100 changes charge by about 4.47e-8 mm). Full-field syntax
rejects ignored suffixes, comma decimals and malformed bytes. Legacy nonfinite
acceleration acceptance is replaced by prior-state preservation and a diagnostic.
Standalone Debug/Release tests qualify these pure numeric/state rules only,
not a real Reader provider, tool activation, buffering, firmware hooks or exports.

## Ordered component assembly

`PressureFlow.hpp` assembles Source, Nominal, Classification, Controls, Planner,
Geometry, Extrusion and Rewrite for one sequential export. It accepts an owned
`SourceFragment` from a completed Reader read, per-line hints and independent
`begin_layer`, `end_layer`, `end_print` flags. The caller copies the previous
Reader final into its next read; Flow does not perform another Reader call.
Resolved settings are copied at construction: tool K/F09, Reader axis letters,
XYZ/E precision, axis limits, volumetric configuration and finite positive initial
acceleration. The caller applies the legacy X fallback (100000 for nonpositive)
and positive maximum-extruding cap before supplying that acceleration.

Begin opens the layer before its text; End closes after it, so both flags can
describe one fragment. EndPrint closes before its text and follows that text
outside layers. Controls and exact nominal coordinates follow the whole input,
including preamble and end scripts. Only eligible in-layer prints are rewritten.
Nominal feed in mm/min is divided by 60 for the planner; emitted F remains nominal.
Effective tool/mode changes close before the same row, whose own eligible motion
can then start the new cycle. K, tool, F09 and F are captured at insertion.
The global persistent override replaces K only for tools configured K>0 or F09;
it can correct an F09-only tool but never activates a neutral tool.

All original planner movements, profiles, junction caps and horizon counters stay
unchanged. Only projected emission groups are merged, before E advances. An
entirely collapsed source transfers forward to the first compatible group; if
that is impossible at an effect or closure, it transfers to the last compatible
still-unemitted group behind it, or fails explicitly. Compatibility requires the
same tool, K, F and E frame, adjacent quantized XYZ and continuous nominal E.
Rebasing at the first part's start is preserved and propagated on a forward
transfer. A rebase at the second part's start would be internal to the merged
group, so that pairing is refused.
Only blank records and comments without Reader or model-control effects may lie
between partners. Fans, TYPE and pressure/acceleration controls prevent material
transfer, even when their values are unchanged, without automatically stopping
the pressure planner. Each collapsed source is rewritten with `rewrite_residue`
after transfer; lost mode/tool effects remain fatal. No E-only compensator is made.
No Reader event is crossed for material transfer, even a SEAM marker that is
transparent to pressure planning.

A merged group takes its starting XYZ, nominal E and speed from its first part,
and its ending values from its last part, with summed distance and duration.
Public geometry phase indices remain local to the emitting anchor. Internal
source-ordinal/phase-range parts retain cross-source provenance. The whole planner
publication or finish remainder is projected and scanned before emission. Flow
retains its last representable source, even when a later last movement collapses;
its E advancement is deferred until that suffix is resolved.

One Extrusion instance per tool persists across cycles, layers and tool returns.
Relative sums start at zero independently of external coordinates and survive
relative G92/SET_POSITION. True E/XYZ/unit mode changes rebase the working frame
at zero charge; inline G90/G91 reaffirmations and their residual parser modes do
not reset rounding. Absolute E reanchors at a proven exact nominal motion start
after an unchanged retract, E-only operation or reset, without calling a semantic
extrusion/retraction operation or reconstructing a nominal snapshot.

`FlowOutput` owns bytes and issues for ignored acceleration, Reader diagnostics or
invalidations, insufficient horizon and merged geometry. Row indices are local;
source ordinals count all physical input rows. Reader issue items are copied.
An insufficient-horizon issue identifies the printing movement whose insertion
triggered that observation, not the first retained movement.
Visible export diagnostic routing is not provided by this assembly. Calls return
no partial output on any exception and a failed instance cannot be reused. Earlier
successful calls are not rolled back; the export owner must not publish its final
file after a failure. There is no historical Planner/Extrusion transaction copy.

Actual emitted text is joined with a single LF when another nonempty text follows
an unterminated row, including across returned fragments. Empty fragments add
nothing; at true EOF the original absence of LF remains. Reader cursors and source
bytes are never changed by this output separation. A record ending in CR without
LF keeps its CR; when another nonempty text follows, the separator LF follows that
CR. No CR is added, removed or moved. At EOF no separator is added.

`pressure_flow` uses local explicit SDK-validated events and existing SDK lexical,
Syntax and numeric components. It does not qualify the real Reader/provider,
firmware hook, configuration lookup, accounting, preview, final metadata separation
or active exports. No runtime is connected here. Those integration boundaries
must preserve the same ordering and failure visibility; they are not inferred
from these standalone assembly tests.
