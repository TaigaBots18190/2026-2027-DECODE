# How we work

This is the whole process. Follow it every time you change the repo.

If a step feels confusing, stop and ask. Do not invent a shortcut.

---

## The five rules

1. **Do not commit to `main`.** `main` is the robot. You work on a branch.
2. **One job per branch.** Intake work and auto work do not share a branch.
3. **Write a sentence for every commit.** Someone else has to understand it next week.
4. **Nothing lands on `main` without a Pull Request and one review.**
5. **If you did not test it, say so in the PR.** That is honest. Hiding it is not.

---

## Step-by-step

Do these in order. The commands assume you already have the repo on your computer.

### 1. Update `main` on your computer

```bash
git checkout main
git pull
```

### 2. Make a branch

Name: `firstname-short-job`

Examples that are good:

- `maya-intake`
- `jordan-teleop-drive`
- `sam-readme-typo`

Examples that are not:

- `fix`
- `new`
- `maya-and-jordan-everything`

```bash
git checkout -b firstname-short-job
```

### 3. Do the work

Change only what this branch is for. Save your files.

### 4. See what you changed

```bash
git status
git diff
```

If you see files you did not mean to touch, stop and ask.

### 5. Commit

```bash
git add -A
git commit -m "Add intake spin-up in teleop."
```

A good commit message is one normal sentence:

- `Fix left-rear motor direction.`
- `Add Red close auto first path.`
- `Correct Pinpoint pod name in Constants.`

Not useful:

- `update`
- `stuff`
- `fixed it`

Small commits are better than one giant commit.

### 6. Put the branch on GitHub

```bash
git push -u origin firstname-short-job
```

### 7. Open a Pull Request

1. Open the repo on GitHub.
2. GitHub will show a banner for your branch. Click **Compare & pull request**.
3. Fill out the form. Every box matters.
4. Request one teammate as a reviewer.

### 8. Review (the teammate)

The reviewer does not need to be a Git expert. They need to answer:

- Do I understand what changed?
- Does this look safe to put on the robot?
- Did they leave a secret, a huge leftover file, or a half-finished experiment?

If something is wrong, write a comment on the PR. The author fixes it on the **same branch** and pushes again. The PR updates by itself.

### 9. Merge

When the reviewer approves:

1. Click **Merge pull request**.
2. Click **Delete branch** when GitHub offers it.
3. On your computer:

```bash
git checkout main
git pull
```

You are done. Next job = new branch from the updated `main`.

---

## What if two people edit the same file?

Do not “force push.” Do not delete `main`.

Pull `main` into your branch and ask a mentor if Git prints a conflict:

```bash
git checkout firstname-short-job
git pull origin main
```

If you see the word **CONFLICT**, stop. A mentor walks through it with you once. After that you can do it yourself.

---

## What never goes in this repo

- Passwords, tokens, or account keys
- Personal signing keystores (`*.jks`, `*.keystore`)
- “Just testing” files on your Desktop that are not part of the robot
- Copied code you cannot explain

The debug keystore that ships with the FTC SDK is different. A mentor will tell you if a file is that one.

---

## Cheatsheet

| I want to… | Do this |
| --- | --- |
| Start work | `git checkout main` then `git pull` then `git checkout -b firstname-job` |
| Save work | `git add -A` then `git commit -m "Sentence here."` |
| Share work | `git push -u origin firstname-job` then open a PR |
| Get latest `main` | `git checkout main` then `git pull` |
| I broke something | Do not keep committing. Ask. |
