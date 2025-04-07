# Progress: COMP0250 Coursework 1 (CW1 Focus)

## Current Status

*   **Phase:** CW1 Development & Refinement.
*   **Overall Progress:** CW1 Tasks 1 & 2 partially implemented (require robustness improvements). CW1 Task 3 not started. CW2 ignored.
*   **Memory Bank:** Updated [2025-04-07] to reflect focus solely on CW1.

## What Works

*   Memory Bank structure and initial documentation are in place and updated for CW1 focus.
*   CW1 Tasks 1 & 2 reportedly function sometimes, indicating basic perception/manipulation logic exists but is unstable.

## What's Left to Build

*   **CW1 Robustness:** Debug and fix initialization crashes for Tasks 1 & 2.
*   **CW1 Task 3:** Implement the full perception and manipulation pipeline:
    *   Detect all colored boxes and baskets.
    *   Identify colors accurately.
    *   Match boxes to corresponding baskets.
    *   Plan and execute pick-and-place for each box into the correct basket.
    *   Ensure collision avoidance.
*   **Refinement:** General code cleanup, optimization, and documentation improvements for the `cw1_team_<team_number>` package.
*   **README:** Update `cw1_team_<team_number>/README.md` with final details, including contributions.

## Known Issues

*   **Initialization Instability:** CW1 Tasks 1 & 2 solutions crash intermittently during initialization. Root cause unknown (suspected timing, race condition, or setup error). [Reported 2025-04-07]

## Project Decisions & Evolution

*   **[2025-04-07]** Initialized Memory Bank structure.
*   **[2025-04-07]** Confirmed location of PDFs in `coursework_description/` directory.
*   **[2025-04-07]** Received user instruction to focus solely on CW1 (robustness for Tasks 1&2, implementation of Task 3) and ignore CW2.
*   **[2025-04-07]** Updated all core Memory Bank files (`projectbrief.md`, `productContext.md`, `activeContext.md`, `systemPatterns.md`, `techContext.md`, `progress.md`) to reflect CW1 focus.

## Next Milestone

*   Diagnose and resolve the initialization stability issues for CW1 Tasks 1 & 2.
*   Begin implementation of the perception component for CW1 Task 3 (detecting multiple boxes/baskets and their colors).
