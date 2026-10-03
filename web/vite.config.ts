import { fileURLToPath } from 'node:url'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
//
// `vite build --mode vscode` builds the page for the VS Code extension: the
// kernel is a native process behind the webview's message channel
// (`backend.vscode.ts`) instead of the wasm module (`backend.ts`), and the
// result goes to the extension's `media/` folder, with relative asset URLs
// so the extension can point them at the webview's own origin.
export default defineConfig(({ mode }) => {
  if (mode !== 'vscode') return { plugins: [react()] }
  return {
    plugins: [react()],
    base: './',
    resolve: {
      alias: [
        {
          find: /^\.\/backend$/,
          replacement: fileURLToPath(new URL('./src/backend.vscode.ts', import.meta.url)),
        },
      ],
    },
    build: {
      outDir: '../vscode-extension/media',
      emptyOutDir: true,
    },
  }
})
