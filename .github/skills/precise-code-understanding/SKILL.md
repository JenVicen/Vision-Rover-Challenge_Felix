---
name: precise-code-understanding
description: 'Use when a coding task must be grounded in verified facts before changing code: non-trivial edits, bug fixes, refactors, unfamiliar or legacy code, ambiguous requirements, or when the user asks for "precise code", "understand first", "best approach", "no guessing", "verify before changing", or "use all available resources". Produces an evidence-backed plan, a minimal correct implementation, and a verification step. DO NOT USE FOR: trivial one-line edits, pure Q&A, or greenfield scaffolding.'
argument-hint: 'Describe the change or bug, and any known constraints'
---

# Precise Code & Understanding

Deliver the best possible outcome per unit of change: understand before editing, use every
cheap source of truth available, and keep the diff minimal and correct.

## When to Use
- Fixing a bug whose root cause is not yet proven.
- Modifying code you have not read in this conversation.
- Choosing between several plausible implementation approaches.
- Any task where a wrong assumption would be expensive to discover later.

## Procedure

### 1. Frame the target (cheap, do not skip)
State in one or two sentences: the observable behavior today, the desired behavior, and the
explicit success criterion (a test, a command output, or a user-visible result).
If the criterion cannot be stated, ask the user before writing any code.

### 2. Ground in facts, not recall
Gather evidence in parallel; stop as soon as the next edit is unambiguous.

| Question | Source of truth |
|---|---|
| Where does this live? | semantic search, then grep for exact symbols |
| What does it actually do? | read the file — full function, not a snippet |
| Who calls it / what breaks? | list code usages (all call sites) |
| Is it already broken? | compile/lint diagnostics for the files in scope |
| What are the real values? | run the code, a test, or the debugger |
| What is the project convention? | a sibling file doing the same thing |
| What does the API really accept? | installed source or official docs — never memory |

Rule: never assert a signature, field name, or behavior you have not seen in this session.
If a claim is load-bearing and unverified, verify it or label it explicitly as an assumption.

### 3. Choose the approach deliberately
List the viable options (usually 2–3) with a one-line trade-off each, then commit to one and
say why. Prefer, in order: reuse existing code → extend it minimally → add a new small unit →
introduce an abstraction (last resort, only with 3+ real call sites).

Reject an approach if it: duplicates existing logic, adds configuration nobody asked for,
changes public behavior not in scope, or cannot be verified by the criterion from step 1.

### 4. Implement minimally
- Change only what the criterion requires. No drive-by refactors, comments, or renames.
- Match surrounding style, error handling, and naming exactly.
- Validate at boundaries only; do not guard against impossible states.
- If the change grows past what you framed in step 1, stop and re-frame.

### 5. Verify, then report
1. Re-check diagnostics on every touched file.
2. Run the success criterion (test, script, or command). Runtime evidence beats reasoning.
3. If it fails, diagnose the cause — do not retry the same approach with variations.
4. Report: what changed, the evidence it works, and any assumption still unverified.

## Completion Checklist
- [ ] Success criterion was stated before the first edit.
- [ ] Every claim about existing code traces to a file read, a search hit, or a run.
- [ ] Alternatives were considered and the choice justified in one line.
- [ ] Diff contains nothing beyond what the criterion requires.
- [ ] Diagnostics clean and the criterion executed, not just assumed.
- [ ] Remaining assumptions and risks stated explicitly.

## Anti-patterns
- Editing a file that was never read in full.
- "This should work" with no execution.
- Broad searching after you already had enough to act.
- Building an abstraction for a single call site.
- Silently widening scope instead of asking.
