#!/usr/bin/env bash
# Publica texto como anotación de GitHub Actions (visible en la pestaña Checks).
annotate() {
  local level="$1" title="$2" body="$3"
  body="${body//'%'/'%25'}"
  body="${body//$'\r'/}"
  body="${body//$'\n'/'%0A'}"
  echo "::${level} title=${title}::${body}"
}
