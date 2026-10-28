#!/usr/bin/env bash
# Runs on the Docker host before the X11 socket and cookie are bind-mounted.
set -euo pipefail
umask 077

display="${1:-${DISPLAY:-}}"
authority="${2:-${XAUTHORITY:-}}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
output="${script_dir}/.xauthority"

if [[ -z "${display}" ]]; then
    echo "X11: DISPLAY is unset on the host. Open VS Code from your graphical session (code .)." >&2
    exit 1
fi
case "${display}" in
    :[0-9]*|unix:[0-9]*) ;;
    *)
        echo "X11: this configuration requires a local X11/XWayland display, such as :0 (got ${display})." >&2
        exit 1
        ;;
esac
display_number="${display#*:}"
display_number="${display_number%%.*}"
if [[ ! -S "/tmp/.X11-unix/X${display_number}" ]]; then
    echo "X11: host socket /tmp/.X11-unix/X${display_number} is missing. Start X11/XWayland first." >&2
    exit 1
fi
if ! command -v xauth >/dev/null 2>&1; then
    echo "X11: install xauth on the host (Ubuntu: sudo apt-get install xauth)." >&2
    exit 1
fi

work_dir="$(mktemp -d)"
trap 'rm -rf -- "${work_dir}"' EXIT
auth_args=()
if [[ -n "${authority}" ]]; then
    auth_args=(-f "${authority}")
fi
xauth "${auth_args[@]}" nlist "${display}" > "${work_dir}/records"
if [[ ! -s "${work_dir}/records" ]]; then
    echo "X11: no authentication cookie found for ${display}. Set XAUTHORITY to the host session's authority file and reopen VS Code." >&2
    exit 1
fi
# FamilyWild makes the selected display's cookie usable with the container hostname.
sed 's/^..../ffff/' "${work_dir}/records" > "${work_dir}/container-records"
touch "${work_dir}/authority"
xauth -f "${work_dir}/authority" nmerge "${work_dir}/container-records"
# Keep the inode so an existing container's read-only bind mount sees refreshed cookies.
cat "${work_dir}/authority" > "${output}"
chmod 600 "${output}"
echo "X11: prepared the authentication cookie for ${display}."
