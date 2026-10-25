import { fileURLToPath } from 'node:url'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

/**
 * The pages whose kernel is a native `geop` process rather than the wasm
 * module (`backend.ts`), by build mode: the backend that replaces it, and
 * where the page goes.
 *
 * - `vite build --mode vscode`: the VS Code extension's page. The kernel is
 *   behind the webview's message channel (`backend.vscode.ts`), and the
 *   page goes to the extension's `media/` folder, with relative asset URLs
 *   so the extension can point them at the webview's own origin.
 * - `vite build --mode open`: the page `geop open` serves, and embeds
 *   (feature `frontend` of `cad/geop-cad-cli`). The kernel is the process
 *   serving it, over a WebSocket (`backend.local.ts`).
 */
const NATIVE: Record<string, { backend: string; outDir: string; base: string }> = {
  vscode: { backend: './src/backend.vscode.ts', outDir: '../vscode-extension/media', base: './' },
  open: { backend: './src/backend.local.ts', outDir: 'dist-open', base: '/' },
}

// https://vite.dev/config/
export default defineConfig(({ mode }) => {
  const native = NATIVE[mode]
  // The kernel's worker (`src/kernel.worker.ts`) is a module: it imports the
  // wasm module's glue.
  if (!native) return { plugins: [react()], worker: { format: 'es' } }
  return {
    plugins: [react()],
    base: native.base,
    resolve: {
      alias: [
        {
          find: /^\.\/backend$/,
          replacement: fileURLToPath(new URL(native.backend, import.meta.url)),
        },
      ],
    },
    build: {
      outDir: native.outDir,
      emptyOutDir: true,
    },
  }
})
