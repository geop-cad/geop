import * as vscode from "vscode";
import { GeopEditorProvider } from "./geopEditor";

export function activate(context: vscode.ExtensionContext) {
  const log = vscode.window.createOutputChannel("Geop");
  context.subscriptions.push(
    log,
    vscode.window.registerCustomEditorProvider(
      GeopEditorProvider.viewType,
      new GeopEditorProvider(context, log),
      // The page keeps the camera and the step being edited, which cannot be
      // rebuilt from the file: keep it alive while its tab is in the background.
      { webviewOptions: { retainContextWhenHidden: true } },
    ),
  );
}

export function deactivate() {}
