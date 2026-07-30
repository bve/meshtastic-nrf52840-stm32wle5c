#!/usr/bin/env bash

set -u
set -o pipefail

readonly upstream_remote="${1:-upstream}"
readonly upstream_branch="${2:-develop}"

fail()
{
    printf 'Ошибка: %s\n' "$1" >&2
    exit 1
}

restore_local_changes()
{
    if [[ "${stash_created}" != true ]]; then
        return 0
    fi

    printf '\nВозвращаю локальные незакоммиченные изменения...\n'
    if git stash pop --index "${stash_ref}"; then
        stash_created=false
        return 0
    fi

    printf '\nНе удалось автоматически применить локальные изменения.\n' >&2
    printf 'Они сохранены в %s. Разрешите конфликты, затем примените stash вручную.\n' "${stash_ref}" >&2
    return 1
}

repo_root="$(git rev-parse --show-toplevel 2>/dev/null)" || fail "откройте task из Git-репозитория Meshtastic."
cd "${repo_root}" || fail "не удалось перейти в ${repo_root}."

current_branch="$(git branch --show-current)"
[[ -n "${current_branch}" ]] || fail "обновление из detached HEAD не поддерживается; сначала переключитесь на рабочую ветку."

git remote get-url "${upstream_remote}" >/dev/null 2>&1 ||
    fail "remote '${upstream_remote}' не найден."

for operation_ref in MERGE_HEAD CHERRY_PICK_HEAD REVERT_HEAD REBASE_HEAD; do
    if git rev-parse --verify -q "${operation_ref}" >/dev/null; then
        fail "сначала завершите или отмените текущую Git-операцию (${operation_ref})."
    fi
done

git_dir="$(git rev-parse --git-dir)"
if [[ -d "${git_dir}/rebase-merge" || -d "${git_dir}/rebase-apply" || -d "${git_dir}/sequencer" ]]; then
    fail "сначала завершите или отмените текущую Git-операцию."
fi

dirty_submodules="$(
    git submodule foreach --quiet --recursive \
        'if test -n "$(git status --porcelain)"; then printf "%s\n" "$displaypath"; fi'
)"
if [[ -n "${dirty_submodules}" ]]; then
    printf 'В подмодулях есть несохранённые изменения:\n%s\n' "${dirty_submodules}" >&2
    fail "сначала закоммитьте или сохраните их внутри соответствующих подмодулей."
fi

stash_created=false
stash_ref='stash@{0}'
if ! git diff --quiet --ignore-submodules=none ||
    ! git diff --cached --quiet --ignore-submodules=none ||
    [[ -n "$(git ls-files --others --exclude-standard)" ]]; then
    stash_message="meshtastic-update-$(date +%Y%m%d-%H%M%S)"
    printf 'Временно сохраняю локальные изменения (%s)...\n' "${stash_message}"
    git stash push --include-untracked --message "${stash_message}" ||
        fail "не удалось сохранить локальные изменения."
    stash_created=true
fi

printf 'Получаю %s/%s (%s)...\n' \
    "${upstream_remote}" \
    "${upstream_branch}" \
    "$(git remote get-url "${upstream_remote}")"
if ! git fetch "${upstream_remote}" "${upstream_branch}"; then
    restore_local_changes
    fail "не удалось получить обновления."
fi

upstream_commit="$(git rev-parse --short FETCH_HEAD)"
printf 'Объединяю %s (%s) с локальной веткой %s...\n' \
    "${upstream_remote}/${upstream_branch}" \
    "${upstream_commit}" \
    "${current_branch}"

if ! git merge --no-edit FETCH_HEAD; then
    printf '\nАвтоматическое объединение не удалось; отменяю незавершённый merge.\n' >&2
    if git rev-parse --verify -q MERGE_HEAD >/dev/null; then
        git merge --abort || fail "не удалось отменить merge; локальные изменения остаются в ${stash_ref}."
    fi
    restore_local_changes
    fail "обновление отменено из-за конфликтов; ваши изменения сохранены."
fi

printf '\nОбновляю подмодули до версий, указанных в прошивке...\n'
submodule_result=0
git submodule update --init --recursive || submodule_result=$?

restore_result=0
restore_local_changes || restore_result=$?

printf '\nТекущая ветка: %s\n' "${current_branch}"
git status --short --branch

if ((submodule_result != 0)); then
    fail "код обновлён, но не удалось обновить один или несколько подмодулей."
fi

if ((restore_result != 0)); then
    exit "${restore_result}"
fi

printf '\nMeshtastic обновлён; локальные коммиты, изменения и вариант сохранены.\n'
