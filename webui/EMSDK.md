# Emscripten for the simulator

Installed version: **6.0.0** (latest at install time), at `C:\emsdk`.

## Activation (Git Bash)

Sourcing `emsdk_env.sh` does **not** put `emcc` on PATH in Git Bash on this
machine. Add the emscripten dir directly:

```bash
export PATH="/c/emsdk/upstream/emscripten:$PATH"
emcc --version   # -> emcc (Emscripten ...) 6.0.0
```

The build script `webui/scripts/build-wasm.sh` does this for you, so normally
you just run:

```bash
bash webui/scripts/build-wasm.sh
```

Override the install location with `EMSDK_DIR=/path/to/emsdk` if it ever moves.

## Notes

- Node ESM on Windows needs a relative or drive-letter `file:///C:/...`
  specifier when importing the generated module; a bare Unix `/tmp/...` path
  errors with `ERR_INVALID_FILE_URL_PATH`.
- The WASM build is emitted as a single self-contained ES module
  (`-sSINGLE_FILE=1 -sMODULARIZE=1 -sEXPORT_ES6=1`), so there is no separate
  `.wasm` asset to locate.
