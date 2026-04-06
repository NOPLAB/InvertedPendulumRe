---
name: commitlint方式のcommitメッセージ
description: Commit messages must follow commitlint / Conventional Commits format
type: feedback
---

commitメッセージはcommitlint（Conventional Commits）方式で書くこと。

形式: `<type>(<scope>): <subject>`

**Why:** ユーザーの明示的な指示。
**How to apply:** すべてのgit commitで `feat`, `fix`, `docs`, `chore`, `refactor`, `test`, `ci` 等のprefixを使う。scopeは任意。subjectは簡潔に。
