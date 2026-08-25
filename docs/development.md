# Development guide

## Working agreement

This repository uses a small team workflow even when developed by one person.
Each change starts from an Issue and ends with a reviewed pull request.

```text
Issue -> branch -> implementation -> tests -> self-review -> PR -> merge
```

One Issue should produce one independently verifiable behavior. Avoid combining
transport, visualization, and recording in one pull request.

## Roles to simulate

- Product owner: defines the user-visible outcome and priority.
- Tech lead: owns architecture and interface decisions.
- Developer: implements an Issue and records assumptions.
- Reviewer: checks correctness, scope, tests, and documentation.
- QA: verifies the Issue's acceptance criteria without relying on internals.

One person can switch roles, but should perform developer and reviewer passes at
different times and explicitly record the review result on the PR.

## Git workflow

Branch names use the Issue number:

```text
feature/<issue-number>-<short-name>
fix/<issue-number>-<short-name>
docs/<issue-number>-<short-name>
```

Commit messages describe one completed change, for example:

```text
docs: define telemetry protocol v1
feat(sender): publish mock pose telemetry
test(receiver): cover coordinate conversion
```

PR descriptions include:

- `Closes #<issue-number>`
- what changed and why;
- how it was tested;
- known limitations or follow-up Issues.

## Definition of Done

An Issue is done when:

- its acceptance criteria are satisfied;
- relevant automated tests pass;
- new public behavior is documented;
- logs contain enough context to diagnose invalid input;
- no unrelated refactoring is included;
- the diff has received a reviewer pass;
- the branch is mergeable.

## Issues and backlog

- #1 Define telemetry Protocol v1, including the coordinate-system contract.
- #4 Implement a Rust Mock Sender for deterministic telemetry.
- #7 Implement the Rust ZeroMQ telemetry subscriber.

Future work receives its number when the GitHub Issue is created:

- Parse and validate Protocol v1.
- Normalize quaternions and preserve sign continuity.
- Convert `slam_world` coordinates to `unity_world`.
- Implement the Unity background subscriber and main-thread queue.
- Render camera pose and a camera frustum.
- Retain and render trajectory history.
- Apply and render point-cloud deltas.
- Record sessions and export the final point cloud as PLY.
- Connect the real SLAM adapter.

Each Issue must leave the system runnable. Until the real SLAM adapter exists,
the Mock Sender is the reference producer used by Receiver and Unity tests.

## Issue 1: Protocol v1

Goal: define a stable contract shared by Sender, Receiver, and Unity.

Acceptance criteria:

- ZeroMQ framing and topics are documented.
- Settings, pose, and point-cloud schemas have valid examples.
- `Twc`, units, axes, handedness, and quaternion order are explicit.
- validation and sequence handling behavior is explicit.
- examples parse as JSON.

## Issue 4: Mock Sender

Goal: publish deterministic telemetry without a camera or SLAM installation.

Proposed behavior:

- bind a configurable PUB endpoint, defaulting to `tcp://*:5555`;
- publish settings repeatedly;
- publish a camera moving in a horizontal circle at a configurable rate;
- periodically add point-cloud fixtures;
- use no random input and support an optional finite duration;
- exit cleanly on Ctrl-C.

Acceptance criteria:

- a packet-dump subscriber sees all three Protocol v1 topics;
- emitted payloads match the examples and validation rules;
- two runs with identical arguments produce identical telemetry;
- unit tests cover sequence numbers and serialization.

Suggested branch:

```text
feature/4-mock-sender
```

## Issue 7: Receiver subscriber

Goal: receive Protocol v1 telemetry from the Mock Sender over ZeroMQ.

Implemented behavior:

- connect a configurable SUB socket, defaulting to
  `tcp://127.0.0.1:5555`;
- subscribe to the `slam/v1/` topic prefix;
- receive topic and JSON payload as two multipart frames;
- reject invalid frame counts, UTF-8, and JSON syntax without
  panicking;
- report received and rejected message counts;
- exit cleanly on Ctrl-C.

Acceptance criteria:

- the Receiver observes settings, pose, and point-cloud topics from the Mock
  Sender;
- malformed multipart messages do not terminate the process;
- the endpoint can be configured from the command line;
- unit tests cover framing and encoding failures.

Suggested branch:

```text
feature/7-receiver-subscriber
```

## Local development order

Run and verify components from left to right:

1. Mock Sender plus packet dump;
2. Mock Sender plus Receiver plus packet dump;
3. Mock Sender plus Receiver plus Unity;
4. real SLAM adapter plus the established pipeline.

This order keeps failures attributable to one boundary at a time.

## Documentation authority

The Markdown files directly under `docs/` and examples directly under
`protocol/` are the current source of truth. The extracted
`docs/slam_remote_viewer_development_docs/` directory and its ZIP are retained
as planning input only; where they differ, the top-level documents take
precedence.
