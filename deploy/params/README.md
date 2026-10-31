Site parameter files for the `edge` service go here. The compose file mounts this folder at `/ws/params`, read-only.
Start from `src/krc_eki_bridge/config/example.yaml`, copy it here as `krc_eki_bridge.yaml`, and fill in the
placeholders. Do not commit files with real addresses: `*.yaml` in this folder is ignored by git.
