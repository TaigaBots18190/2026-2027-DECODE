# For mentors

Students follow [HOW_WE_WORK.md](HOW_WE_WORK.md). This page is the short list of GitHub settings that make that page true.

Do this once. Students should not have to think about it.

## Protect `main`

In the GitHub repo: **Settings → Rules → Rulesets** (or **Settings → Branches**).

Turn on a rule for `main`:

1. Require a pull request before merging.
2. Require **1** approval.
3. Do not allow direct pushes to `main` (except admins, if you want an emergency hatch).
4. Do not require status checks until we add them later.
5. Do not require a dozen reviewers. One teammate is the bar.

That is enough. More rules will sit unused and then get turned off.

## Who can merge

Give every student in the programming group **Write** access so they can push branches and open PRs.

Keep **Admin** for mentors only.

## What not to add yet

- Required linear history
- Required rebase
- Lots of CODEOWNERS paths
- A bot that nags on every comma

Those are later, if ever.

## When a student is stuck

The usual failures are: committed to `main` locally, created a branch from old `main`, or tried to fix a conflict by deleting files.

Walk through the commands on [HOW_WE_WORK.md](HOW_WE_WORK.md) with them once, on their machine. Do not take the laptop and finish it for them unless the robot is on the line tomorrow.
