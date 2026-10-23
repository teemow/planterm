# planterm: notes for agents

planterm is a header-only C++17 library for CAREL's pLAN terminal protocol. [README.md](README.md) describes the headers, the tests and the optional ESPHome bridge; [docs/protocol.md](docs/protocol.md) is the protocol as reverse-engineered; [docs/capture-protocol.md](docs/capture-protocol.md) is the bridge's capture stream.

## The library boundary

- The library has **no ESPHome dependency**, by default and by design: platform-agnostic, bytes and the 9th bit in, TX actions out, time injected. Keep ESPHome (and any other framework) types out of `src/`, the library manifests and the published package; `library.json`'s `export.exclude` keeps `esphome/` and `planterm/` out of the registry package.
- `esphome/components/plan_bridge/` stays in this repository only as a strictly optional extra for `external_components` consumers. Nothing in the library, the examples or the host tests may require it.
- Everything about one installation (a device's page and menu tables, its macros, Home Assistant entities, one person's setup) belongs to the firmware that consumes planterm, not here. The library ships matchers and engines; the tables are the consumer's data.
- `PlanTerminal` (`src/plan_terminal.h`) is frozen in the shape that has run on a live bus for weeks; read its inline comments before restructuring anything.

## This repository is public

Write files, commits, pull requests and issues for a stranger: no private repositories, hostnames, cluster or home-network details, and nothing about anyone's installation beyond what the reference hardware in the README already says.

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
