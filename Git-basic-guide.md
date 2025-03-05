# Git Training Quick Reference

These notes were taken during a GitHub training session by **Gerard Capes** and **Carri King**. Use this guide to help you with daily Git operations while working on your projects.

---

## Table of Contents

1. [Git Configuration](#git-configuration)
2. [Basic Navigation & Repository Initialization](#basic-navigation--repository-initialization)
3. [Editing Files with Nano](#editing-files-with-nano)
4. [Status, Staging, and Committing](#status-staging-and-committing)
5. [Tracking Changes & Diffs](#tracking-changes--diffs)
6. [Logs and Commit Navigation](#logs-and-commit-navigation)
7. [Tagging and Branching](#tagging-and-branching)
8. [Detached HEAD & Branching Off](#detached-head--branching-off)
9. [Undoing Changes & Amending Commits](#undoing-changes--amending-commits)
10. [GitHub Integration & Remote Repositories](#github-integration--remote-repositories)
11. [Additional Tools and Commands](#additional-tools-and-commands)
12. For Doctor = https://uomresearchit.github.io/docker-introduction/
13. https://chryswoods.com/main/courses.html for all the courses on parallel computing too
---

## Git Configuration

Set up global Git settings:

```bash
git config --global user.name "sinannaseel"
git config --global user.email "-----------@gmail.com"
git config --global pull.rebase false
```

---

## Basic Navigation & Repository Initialization

- **Print Current Directory:**
  ```bash
  pwd  # Displays your current directory
  ```
- **Create and Enter a New Directory:**
  ```bash
  mkdir <directory_name>
  cd <directory_name>
  ```
- **Initialize a New Git Repository:**
  ```bash
  git init
  ```
- **View Local Branches:**
  ```bash
  git branch
  ```
- **Git Configuration Help:**
  ```bash
  git config --help
  ```
 This might not work at times, you might have to install some packages 
---

## Editing Files with Nano

- **Create or Edit a File:**
  ```bash
  nano paper.md
  ```
  - Type your content.
  - Press `Ctrl + O` to save, then `Enter` to confirm.
  - Press `Ctrl + X` to exit nano.

---

## Status, Staging, and Committing

- **Check Repository Status:**
  ```bash
  git status
  ```
- **Stage a File:**
  ```bash
  git add paper.md
  ```
- **Commit Changes:**
  ```bash
  git commit -m "Commit message here"
  ```

---

## Tracking Changes & Diffs

- **View Unstaged Changes:**
  ```bash
  git diff
  ```
- **Use a Diff Tool Interface:**
  ```bash
  git difftool
  ```
- **Commit All Changes:**
  ```bash
  git commit -a -m "Commit message here"
  ```

---

## Logs and Commit Navigation

- **View Commit History:**
  ```bash
  git log
  ```
- **Compact Graphical Log:**
  ```bash
  git log --graph --decorate --oneline --all
  ```
- **Switch to a Specific Commit:**
  ```bash
  git switch -d <commit_id>
  ```
- **Return to Master Branch:**
  ```bash
  git switch master
  ```

---

## Tagging and Branching

- **Tag a Commit:**
  ```bash
  git tag PAPER_STUD
  git tag  # Lists all tags
  ```
- **Switch to a Tag:**
  ```bash
  git switch -d PAPER_STUD
  ```
- **Create and Switch to a New Branch:**
  ```bash
  git branch simulations
  git switch simulations
  ```
- **Merge a Branch:**
  ```bash
  git merge simulations
  ```

---

## Detached HEAD & Branching Off

- **Creating a New Branch from Detached HEAD:**
  ```bash
  git switch -c dh-exercise
  ```

---

## Undoing Changes & Amending Commits

- **Restore Changes Before Staging:**
  ```bash
  git restore <file_name>
  ```
- **Amend Last Commit:**
  ```bash
  git commit --amend
  ```
- **Undo a Commit:**
  ```bash
  git revert <commit_id>
  ```
- **Hard Reset (Warning: This can lose changes):**
  ```bash
  git reset --hard <commit_id>
  ```

---

## GitHub Integration & Remote Repositories

- **Add a Remote Repository:**
  ```bash
  git remote add origin git@github.com:sinannaseel/paper.git
  ```
- **Push Changes to Remote:**
  ```bash
  git push -u origin master
  ```
- **Push a Specific Branch:**
  ```bash
  git push origin simulations
  ```
- **Push All Branches:**
  ```bash
  git push --all
  ```
- **Clone a Repository:**
  ```bash
  git clone git@github.com:sinannaseel/paper.git laptop_paper
  ```

---

## Additional Tools and Commands

- **View the Git Manual:**
  ```bash
  man git
  ```
- **Best Practices:**
  - Avoid rewriting history on shared branches.
  - When in doubt, create new commits instead of amending or rebasing published work.

---

## Instructors

- **Gerard Capes**
- **Carri King**

---

*End of Git Training Quick Reference*


