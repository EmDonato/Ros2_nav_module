#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    echo "Usage: $0 COMMAND"
    echo "Commands:"
    echo "  build            Build the image"
    echo "  create,  -c      Build, create and start the container"
    echo "  start,   -s      Start the existing container"
    echo "  stop,    -t      Stop the container"
    echo "  restart, -r      Restart the container and reload parameters"
    echo "  remove,  -x      Stop and remove the container"
    echo "  status           Show the service state"
    echo "  logs             Follow the container logs"
    echo "  check            Check Docker and module prerequisites"
    echo "  help,    -h      Show help"
}

compose() {
    docker compose --file "${SCRIPT_DIR}/compose.yaml" "$@"
}

check() {
    command -v docker >/dev/null || { echo "[ERROR] Docker is not installed" >&2; return 1; }
    docker compose version >/dev/null
    [[ -r "${SCRIPT_DIR}/config/navigation.yaml" ]] || {
        echo "[ERROR] Configuration is not readable: ${SCRIPT_DIR}/config/navigation.yaml" >&2
        return 1
    }
    compose config --quiet
    docker info >/dev/null
}

[[ $# -eq 1 ]] || { usage; exit 2; }
case "$1" in
    help|-h|--help) usage; exit 0 ;;
    build|--build) action=build ;;
    create|-c|--create) action=create ;;
    start|-s|--start) action=start ;;
    stop|-t|--stop) action=stop ;;
    restart|-r|--restart) action=restart ;;
    remove|-x|--remove) action=remove ;;
    status|--status) action=status ;;
    logs|--logs) action=logs ;;
    check|--check) action=check ;;
    *) usage; exit 2 ;;
esac

check
case "${action}" in
    build) compose build navigation ;;
    create) compose up --detach --build navigation ;;
    start) compose start navigation ;;
    stop) compose stop navigation ;;
    restart) compose restart navigation ;;
    remove) compose rm --stop --force navigation ;;
    status) compose ps navigation ;;
    logs) compose logs --follow navigation ;;
    check) echo "[OK] Docker, Compose and navigation configuration are available" ;;
esac
