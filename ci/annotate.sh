#!/usr/bin/env bash
# Publica texto como anotación de GitHub Actions (visible en la pestaña Checks).
annotate() {
  local level="$1" title="$2" body="$3"
  body="${body//'%'/'%25'}"
  body="${body//$'\r'/}"
  body="${body//$'\n'/'%0A'}"
  echo "::${level} title=${title}::${body}"
}

stop_group() {
  local pid="$1"
  kill -INT -- -"$pid" 2>/dev/null
  for _ in $(seq 1 20); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 1
  done
  kill -KILL -- -"$pid" 2>/dev/null
  sleep 1
}
