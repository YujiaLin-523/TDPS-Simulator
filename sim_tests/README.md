# Simulation Test Layout

`sim_tests/` hosts offline simulation test code and scenario suites.

## Structure

```text
sim_tests/
├── common/
│   └── harness/               # reusable test harness code
└── line_follow_v1/
    ├── config/                # scenario CSVs
    ├── lf_autotest_harness.c  # suite entrypoint
    └── README.md
```

## Extension Strategy

When adding new algorithm iterations:

1. Create a new suite folder next to `line_follow_v1/`.
2. Reuse `common/harness/` unless core simulation behavior changes.
3. Keep all outputs in `../artifacts/<suite_name>/`.
