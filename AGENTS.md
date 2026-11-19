# Project conventions

- Scope: XGO Rider only. Do not add another robot or attribute unverified
  requirements to the professor.
- Write all code in English, including identifiers, comments, test names,
  console commands, messages, event names, and CSV columns.
- Keep conditions short. Use guard clauses and small functions that express
  a meaningful rule instead of long boolean expressions or deep nesting.
- Keep console command dispatch separate from command handlers. Keep protocol
  validation, state transitions, and IPC lifecycle responsibilities distinct.
- Maintain `src/`, `include/`, `examples/`, `tests/`, and `docs/`. Generated
  executables belong in `build/`.
- Documentation and user-facing explanations may remain in Spanish. Update
  command examples and CSV field descriptions when the interface changes.
- Preserve stop, explicit arm, watchdog, and shutdown behavior when refactoring.
  Validate C11 compilation with warnings treated as errors and run `make test`
  in Linux for changes to the concurrent simulator or protocol.
